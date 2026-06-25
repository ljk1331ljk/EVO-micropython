import bluetooth
import micropython
import ujson
import ubinascii
import utime
import machine
import gc

try:
    import uos as os
except ImportError:
    import os

from micropython import const


# ============================================================================
# Easy BLE size setting
# ============================================================================

MTU_SIZE = 20

NOTIFY_CHUNK_DELAY_MS = 8
NOTIFY_YIELD_EVERY = 4
# DATA frame format is:
#   4 bytes seq + 4 bytes offset + payload
DATA_HEADER_SIZE = 8
DATA_PAYLOAD_SIZE = max(1, MTU_SIZE - DATA_HEADER_SIZE)


# ============================================================================
# Config
# ============================================================================

# Single shared Evo config file.
# This file is created/owned by mod_evo.c so each firmware variant can provide
# its own default name, controller_type, and BLE download defaults.
CONFIG_PATH = "/evo_config.json"
DOWNLOAD_SECTION = "download"
LEGACY_BLE_DOWNLOAD_SECTION = "ble_download"
PROGRAMS_ROOT = "/programs"
PROGRAM_MAIN_FILE = "main.py"
PROGRAM_DOWNLOAD_COUNTER_FILE = ".evo_download_counter"
PROGRAM_DOWNLOAD_ORDER_FILE = ".evo_download_order"

_DEFAULT_BLE_DOWNLOAD_CFG = {
    "start_on_boot": True,
    "adv_interval_us": 200000,
    "debug": False,
    "ack_every": 1,
    "sensor_streaming": True,
    "sensor_tick_ms": 50,
}

_DEFAULT_ROOT_CFG = {
    "name": "Evo",
    "controller_type": "UNKNOWN",
    "multiple_program_filesystem": False,
    "bluetooth_enabled": True,
    DOWNLOAD_SECTION: _DEFAULT_BLE_DOWNLOAD_CFG,
}

# /boot.py is intentionally NOT protected.
# The PWA may upload /boot.py if required.
_PROTECTED_PATHS = (
    "/evo_config.json",
)

_MAX_CONTROL_QUEUE = const(16)
_MAX_DATA_QUEUE = const(32)

_cfg_cache = None
_service_instance = None


# ============================================================================
# Config helpers
# ============================================================================

def _copy_dict(d):
    out = {}
    for k in d:
        v = d[k]
        if isinstance(v, dict):
            out[k] = _copy_dict(v)
        else:
            out[k] = v
    return out


def _root_cfg_default():
    return _copy_dict(_DEFAULT_ROOT_CFG)


def _root_cfg_normalise(root):
    if not isinstance(root, dict):
        root = {}

    cfg = _root_cfg_default()

    # User-editable global settings.
    if "name" in root:
        cfg["name"] = str(root.get("name", "Evo"))[:24]

    # Read-only/fixed identity created by mod_evo.c. EvoDownloadManager never
    # writes this unless it is creating an emergency fallback config.
    if "controller_type" in root:
        cfg["controller_type"] = str(root.get("controller_type", "UNKNOWN"))
    elif "device_type" in root:
        cfg["controller_type"] = str(root.get("device_type", "UNKNOWN"))

    if "multiple_program_filesystem" in root:
        cfg["multiple_program_filesystem"] = bool(
            root.get("multiple_program_filesystem", False)
        )

    if "bluetooth_enabled" in root:
        cfg["bluetooth_enabled"] = bool(root.get("bluetooth_enabled", True))

    section = root.get(DOWNLOAD_SECTION, root.get(LEGACY_BLE_DOWNLOAD_SECTION, {}))
    if not isinstance(section, dict):
        section = {}

    ble = _copy_dict(_DEFAULT_BLE_DOWNLOAD_CFG)
    for k in section:
        if k in ble:
            ble[k] = section[k]

    try:
        ble["start_on_boot"] = bool(ble.get("start_on_boot", True))
        ble["adv_interval_us"] = int(ble.get("adv_interval_us", 200000))
        ble["debug"] = bool(ble.get("debug", False))
        ble["ack_every"] = max(1, int(ble.get("ack_every", 1)))
        ble["sensor_streaming"] = bool(ble.get("sensor_streaming", True))
        ble["sensor_tick_ms"] = max(20, int(ble.get("sensor_tick_ms", 50)))
    except Exception:
        ble = _copy_dict(_DEFAULT_BLE_DOWNLOAD_CFG)

    cfg[DOWNLOAD_SECTION] = ble
    return cfg


def _root_cfg_load():
    try:
        with open(CONFIG_PATH, "r") as f:
            raw = f.read() or "{}"
            root = ujson.loads(raw)
    except Exception:
        root = _root_cfg_default()
        _root_cfg_save(root)

    return _root_cfg_normalise(root)


def _root_cfg_save(root):
    tmp = CONFIG_PATH + ".tmp"

    try:
        with open(tmp, "w") as f:
            f.write(ujson.dumps(root))

        try:
            os.remove(CONFIG_PATH)
        except Exception:
            pass

        try:
            os.rename(tmp, CONFIG_PATH)
        except Exception:
            with open(CONFIG_PATH, "w") as f:
                f.write(ujson.dumps(root))
            try:
                os.remove(tmp)
            except Exception:
                pass

        return True

    except Exception:
        return False


def _cfg_load():
    global _cfg_cache

    if _cfg_cache is not None:
        return _cfg_cache

    root = _root_cfg_load()
    ble = root.get(DOWNLOAD_SECTION, root.get(LEGACY_BLE_DOWNLOAD_SECTION, {}))

    cfg = _copy_dict(_DEFAULT_BLE_DOWNLOAD_CFG)
    for k in ble:
        if k in cfg:
            cfg[k] = ble[k]

    # Expose shared top-level values as read-only context to this module.
    cfg["name"] = str(root.get("name", "Evo"))[:24]
    cfg["controller_type"] = str(root.get(
        "controller_type",
        root.get("device_type", "UNKNOWN"),
    ))
    cfg["multiple_program_filesystem"] = bool(
        root.get("multiple_program_filesystem", False)
    )
    cfg["bluetooth_enabled"] = bool(root.get("bluetooth_enabled", True))

    _cfg_cache = cfg
    return cfg


def _cfg_save(cfg=None):
    global _cfg_cache

    root = _root_cfg_load()

    if cfg is None:
        cfg = _cfg_load()

    # Only write BLE download settings here. Top-level name and other shared
    # settings should normally be managed by mod_evo.c, but set_name_persistent()
    # below is kept as a convenience for this module's CLI.
    ble = root.get(DOWNLOAD_SECTION, root.get(LEGACY_BLE_DOWNLOAD_SECTION, {}))
    if not isinstance(ble, dict):
        ble = {}

    for k in _DEFAULT_BLE_DOWNLOAD_CFG:
        if k in cfg:
            ble[k] = cfg[k]

    root[DOWNLOAD_SECTION] = ble

    ok = _root_cfg_save(root)
    if ok:
        _cfg_cache = None
        _cfg_cache = _cfg_load()

    return ok


def config_get():
    return _cfg_load()


def config_set(**kwargs):
    cfg = _cfg_load()

    for k, v in kwargs.items():
        if k in _DEFAULT_BLE_DOWNLOAD_CFG:
            cfg[k] = v

    _cfg_save(cfg)
    return _cfg_load()


def enable_persistent(on=True):
    root = _root_cfg_load()
    root["bluetooth_enabled"] = bool(on)
    ok = _root_cfg_save(root)
    if ok:
        global _cfg_cache
        _cfg_cache = None
    return ok


def start_on_boot_persistent(on=True):
    return config_set(start_on_boot=bool(on))


def set_name_persistent(name):
    global _cfg_cache

    root = _root_cfg_load()
    root["name"] = str(name)[:24]
    _root_cfg_save(root)
    _cfg_cache = None
    cfg = _cfg_load()

    if _service_instance is not None:
        try:
            _service_instance.set_name(cfg["name"])
        except Exception:
            pass

    return cfg


def set_ack_every_persistent(n=1):
    return config_set(ack_every=max(1, int(n)))


def sensor_streaming_persistent(on=True):
    return config_set(sensor_streaming=bool(on))


def set_sensor_tick_persistent(ms=50):
    return config_set(sensor_tick_ms=max(20, int(ms)))

# ============================================================================
# Utility helpers
# ============================================================================

def _log(enabled, *args):
    if enabled:
        try:
            print("[EvoDownloadManager]", *args)
        except Exception:
            pass


def _safe_str_preview(s, max_len=200):
    if s is None:
        return ""

    s = str(s)

    if len(s) > max_len:
        return s[:max_len] + "..."

    return s


def _ba_find(ba, sub):
    try:
        return ba.find(sub)
    except Exception:
        return bytes(ba).find(sub)


def _ba_consume(ba, n):
    if n <= 0:
        return ba

    if n >= len(ba):
        return bytearray()

    return bytearray(ba[n:])


def _normalise_path(path):
    if path is None:
        return None

    path = str(path).strip()
    path = path.replace("\\", "/")

    while "//" in path:
        path = path.replace("//", "/")

    if not path:
        return None

    if not path.startswith("/"):
        path = "/" + path

    parts = []

    for p in path.split("/"):
        if not p or p == ".":
            continue

        if p == "..":
            return None

        parts.append(p)

    return "/" + "/".join(parts)


def _is_root_file(path):
    path = _normalise_path(path)

    if not path:
        return False

    if path == "/":
        return False

    return path.count("/") == 1


def _is_py_file(path):
    path = _normalise_path(path)

    if not path:
        return False

    return path.endswith(".py")


def _is_safe_path_part(part):
    if not part:
        return False

    for ch in str(part):
        if not (
            ("a" <= ch <= "z")
            or ("A" <= ch <= "Z")
            or ("0" <= ch <= "9")
            or ch in ("_", "-", ".")
        ):
            return False

    return True


def _programs_enabled():
    try:
        return bool(config_get().get("multiple_program_filesystem", False))
    except Exception:
        return False


def _is_program_file(path):
    path = _normalise_path(path)

    if not path or not _programs_enabled():
        return False

    prefix = PROGRAMS_ROOT + "/"
    if not path.startswith(prefix):
        return False

    rel = path[len(prefix):]
    parts = rel.split("/")

    if len(parts) < 2:
        return False

    for part in parts:
        if not _is_safe_path_part(part):
            return False

    return _is_py_file(path)


def _program_name_from_path(path):
    path = _normalise_path(path)

    if not path:
        return None

    prefix = PROGRAMS_ROOT + "/"
    if not path.startswith(prefix):
        return None

    rel = path[len(prefix):]
    parts = rel.split("/")

    if len(parts) < 2 or not _is_safe_path_part(parts[0]):
        return None

    return parts[0]


def _upload_allowed(path):
    path = _normalise_path(path)

    if not path:
        return False

    if _is_protected_path(path):
        return False

    if _is_root_file(path) and _is_py_file(path):
        return True

    return _is_program_file(path)


def _is_protected_path(path):
    path = _normalise_path(path)

    if not path:
        return True

    if path in _PROTECTED_PATHS:
        return True

    if path.endswith(".download.tmp"):
        return True

    return False


def _validate_upload_path(path):
    path = _normalise_path(path)

    if not path:
        return False, "BAD_PATH"

    if not _is_py_file(path):
        return False, "PY_ONLY"

    if _is_protected_path(path):
        return False, "PROTECTED"

    if _is_root_file(path):
        return True, path

    if _is_program_file(path):
        return True, path

    if _programs_enabled():
        return False, "PROGRAM_PATH"

    if not _is_root_file(path):
        return False, "ROOT_ONLY"

    return True, path


def _tmp_path_for(path):
    return path + ".download.tmp"


def _stat_path(path):
    try:
        return os.stat(path)
    except Exception:
        return None


def _is_dir_from_stat(st):
    if not st:
        return False

    try:
        return (st[0] & 0x4000) != 0
    except Exception:
        return False


def _ensure_parent_dirs(path):
    path = _normalise_path(path)

    if not path:
        return

    parts = path.split("/")[1:-1]
    cur = ""

    for part in parts:
        cur += "/" + part

        try:
            os.mkdir(cur)
        except Exception:
            pass


def _list_dir(path="/"):
    path = _normalise_path(path) or "/"
    items = []

    try:
        names = os.listdir(path)
    except Exception as e:
        return [{
            "name": path,
            "path": path,
            "type": "error",
            "error": str(e),
        }]

    for name in names:
        try:
            if name in (PROGRAM_DOWNLOAD_COUNTER_FILE, PROGRAM_DOWNLOAD_ORDER_FILE):
                continue

            full = (path.rstrip("/") + "/" + name) if path != "/" else "/" + name

            if name.endswith(".download.tmp"):
                continue

            st = _stat_path(full)
            is_dir = _is_dir_from_stat(st)

            if is_dir:
                items.append({
                    "name": name,
                    "path": full,
                    "type": "dir",
                    "upload_allowed": False,
                    "protected": False,
                })
            else:
                size = 0

                try:
                    if st and len(st) > 6:
                        size = int(st[6])
                except Exception:
                    pass

                protected = _is_protected_path(full)

                items.append({
                    "name": name,
                    "path": full,
                    "type": "file",
                    "size": size,
                    "upload_allowed": _upload_allowed(full),
                    "protected": protected,
                })

        except Exception as e:
            items.append({
                "name": str(name),
                "type": "error",
                "error": str(e),
            })

    return items


def _list_root():
    return _list_dir("/")


def _program_capabilities():
    supported = _programs_enabled()

    return {
        "supported": supported,
        "root": PROGRAMS_ROOT,
        "main_file": PROGRAM_MAIN_FILE,
        "launcher": "/main.py",
    }


def _program_download_counter_path():
    return PROGRAMS_ROOT + "/" + PROGRAM_DOWNLOAD_COUNTER_FILE


def _program_download_order_path(program_name):
    return PROGRAMS_ROOT + "/" + program_name + "/" + PROGRAM_DOWNLOAD_ORDER_FILE


def _read_int_file(path, default=0):
    try:
        with open(path) as f:
            return int(f.read().strip() or default)
    except Exception:
        return default


def _write_text_file(path, value):
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return True
    except Exception:
        return False


def _mark_program_download(path):
    program_name = _program_name_from_path(path)

    if program_name is None:
        return

    try:
        os.mkdir(PROGRAMS_ROOT)
    except Exception:
        pass

    counter_path = _program_download_counter_path()
    order = _read_int_file(counter_path, 0) + 1

    _write_text_file(counter_path, order)
    _write_text_file(_program_download_order_path(program_name), order)


def _file_crc32(path):
    crc = 0

    with open(path, "rb") as f:
        while True:
            buf = f.read(1024)

            if not buf:
                break

            crc = ubinascii.crc32(buf, crc)

    return crc & 0xFFFFFFFF


def _safe_replace(tmp_path, final_path):
    try:
        os.remove(final_path)
    except Exception:
        pass

    os.rename(tmp_path, final_path)


def _u32le(b, i=0):
    return (
        b[i]
        | (b[i + 1] << 8)
        | (b[i + 2] << 16)
        | (b[i + 3] << 24)
    )


def _crc32_u32(data_bytes):
    return ubinascii.crc32(data_bytes) & 0xFFFFFFFF


# ============================================================================
# BLE advertising helpers
# ============================================================================

_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)

_FLAG_GENERAL_DISC = const(0x02)
_FLAG_BREDR_NOT_SUPPORTED = const(0x04)


def _adv_payload_name_only(name=None, max_name_len=12):
    payload = bytearray()

    payload += bytes((
        2,
        _ADV_TYPE_FLAGS,
        _FLAG_GENERAL_DISC | _FLAG_BREDR_NOT_SUPPORTED,
    ))

    if name:
        n = str(name).encode()

        if len(n) > max_name_len:
            n = n[:max_name_len]

        payload += bytes((len(n) + 1, _ADV_TYPE_NAME)) + n

    return payload


def _scan_resp_payload_services(services=None):
    payload = bytearray()

    if services:
        for uuid in services:
            b = bytes(uuid)

            if len(b) == 16:
                payload += bytes((len(b) + 1, _ADV_TYPE_UUID128_COMPLETE)) + b

    return payload


SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
CONTROL_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
DATA_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
NOTIFY_UUID = bluetooth.UUID("6E400004-B5A3-F393-E0A9-E50E24DCCA9E")

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)


# ============================================================================
# Sensor helpers
# ============================================================================

_SENSOR_MODULES = {
    "tof": ("EvoTOF", "EvoTOF"),
    "color": ("EvoColorSensor", "EvoColorSensor"),
    "lineleader": ("EvoLineLeader", "EvoLineLeader"),
    "imu": ("EvoIMU", "EvoIMU"),
    "huskylens": ("EvoHuskyLens", "EvoHuskyLens"),
    "tributton": ("EvoTriButton", "EvoTriButton"),
}

_SENSOR_ALIASES = {
    "distance": "tof",
    "vl53l0x": "tof",
    "colorsensor": "color",
    "line": "lineleader",
    "line_leader": "lineleader",
    "bno055": "imu",
    "button": "tributton",
    "buttons": "tributton",
}


def _normalise_sensor_name(sensor):
    if sensor is None:
        return None

    s = str(sensor).strip().lower()
    return _SENSOR_ALIASES.get(s, s)


def _import_attr(module_name, attr_name):
    mod = __import__(module_name)
    return getattr(mod, attr_name)


def _construct_sensor(sensor, port=None, options=None):
    sensor = _normalise_sensor_name(sensor)

    if sensor not in _SENSOR_MODULES:
        raise ValueError("unsupported sensor: " + str(sensor))

    module_name, class_name = _SENSOR_MODULES[sensor]
    cls = _import_attr(module_name, class_name)

    options = options or {}

    if port is not None and options:
        return cls(port, **options)

    if port is not None:
        return cls(port)

    if options:
        return cls(**options)

    return cls()


def _read_sensor(sensor, obj):
    if not hasattr(obj, "read"):
        raise AttributeError(str(sensor) + " sensor does not have read()")

    data = obj.read()

    if not isinstance(data, dict):
        raise TypeError(str(sensor) + ".read() must return dict")

    return data


# ============================================================================
# EvoDownloadManager
# ============================================================================

class EvoDownloadManager:
    def __init__(self, name="Evo", debug=False):
        self._name = str(name)[:24]
        self._debug = bool(debug)

        cfg = config_get()

        self._ack_every = max(1, int(cfg.get("ack_every", 1)))
        self._sensor_streaming_enabled = bool(cfg.get("sensor_streaming", True))
        self._sensor_tick_ms = max(20, int(cfg.get("sensor_tick_ms", 50)))

        self._ble = bluetooth.BLE()

        try:
            self._ble.active(True)
        except Exception:
            pass

        # If the MicroPython build supports MTU config, this will attempt it.
        # If unsupported, the manager still uses MTU_SIZE for safe chunking.
        try:
            self._ble.config(mtu=MTU_SIZE)
        except Exception:
            pass

        self._ble.irq(self._irq)

        self._control = (
            CONTROL_UUID,
            bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE,
        )

        self._data = (
            DATA_UUID,
            bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE,
        )

        self._notify = (
            NOTIFY_UUID,
            bluetooth.FLAG_NOTIFY,
        )

        ((self._h_control, self._h_data, self._h_notify),) = self._ble.gatts_register_services((
            (
                SERVICE_UUID,
                (
                    self._control,
                    self._data,
                    self._notify,
                ),
            ),
        ))

        self._conn_handle = None

        self._adv_interval_us = int(cfg.get("adv_interval_us", 200000))
        self._adv_payload = _adv_payload_name_only(name=self._name, max_name_len=12)
        self._resp_payload = _scan_resp_payload_services(services=[SERVICE_UUID])

        self._control_frag_q = []
        self._data_q = []
        self._work_scheduled = False
        self._queue_overflow_reported = False

        self._ctrl_buf = bytearray()
        self._ctrl_last_rx_ms = utime.ticks_ms()
        self._CTRL_MAX_BUF = 4096
        self._CTRL_IDLE_RESET_MS = 4000

        self._supports = [
            "HELLO",
            "INFO",
            "LIST",
            "CRC32",
            "RESUME",
            "PUT_BEGIN",
            "PUT_END",
            "ABORT",
            "RESET",
            "ROOT_ONLY",
            "PY_ONLY",
            "MAIN_PY",
            "MULTIPLE_PROGRAM_FILESYSTEM",
            "PROGRAM_FOLDERS",
            "LIST_ALL",
            "TEMP_RENAME",
            "PATH_PROTECT",
            "QUEUE_LIMIT",
            "CONSOLE_OUTPUT",
            "SENSOR_LIST",
            "SENSOR_READ",
            "SENSOR_STREAM_START",
            "SENSOR_STREAM_STOP",
        ]

        self._put_active = False
        self._put_path = None
        self._put_tmp_path = None
        self._put_size = 0
        self._put_fp = None
        self._put_next_offset = 0
        self._put_crc32_chunks = False
        self._put_expected_crc32 = None
        self._put_received = 0
        self._put_last_ack_seq = -1

        self._sensor_streams = {}
        self._sensor_timer = None
        self._sensor_work_scheduled = False

        _log(self._debug, "initialised", self._name, "MTU_SIZE", MTU_SIZE)

    # -------------------------------------------------------------------------
    # BLE advertise
    # -------------------------------------------------------------------------

    def start_advertising(self, interval_us=None):
        if interval_us is None:
            interval_us = self._adv_interval_us
        else:
            self._adv_interval_us = int(interval_us)

        try:
            self._ble.gap_advertise(None)
        except Exception:
            pass

        self._ble.gap_advertise(
            self._adv_interval_us,
            adv_data=self._adv_payload,
            resp_data=self._resp_payload,
        )

        _log(self._debug, "advertising", self._adv_interval_us)

    def stop_advertising(self):
        try:
            self._ble.gap_advertise(None)
        except Exception:
            pass

    def set_name(self, name):
        self._name = str(name)[:24]
        self._adv_payload = _adv_payload_name_only(name=self._name, max_name_len=12)

        if self._conn_handle is None:
            try:
                self.start_advertising(self._adv_interval_us)
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # IRQ and queue handling
    # -------------------------------------------------------------------------

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, addr_type, addr = data
            self._conn_handle = conn_handle
            self.stop_advertising()
            self._queue_overflow_reported = False

            self._notify_json({
                "op": "EVENT",
                "type": "CONNECTED",
            })

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, addr_type, addr = data

            if self._conn_handle == conn_handle:
                self._conn_handle = None

            self._abort_put("DISCONNECTED", notify=False, remove_tmp=False)
            self._sensor_stream_stop({})
            self._ctrl_buf = bytearray()
            self._control_frag_q = []
            self._data_q = []

            try:
                if config_get().get("bluetooth_enabled", True):
                    self.start_advertising(self._adv_interval_us)
            except Exception:
                pass

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data

            if value_handle == self._h_control:
                raw = self._ble.gatts_read(self._h_control)

                if len(self._control_frag_q) >= _MAX_CONTROL_QUEUE:
                    self._report_queue_overflow("CONTROL")
                    return

                self._control_frag_q.append(raw)
                self._schedule_work()

            elif value_handle == self._h_data:
                raw = self._ble.gatts_read(self._h_data)

                if len(self._data_q) >= _MAX_DATA_QUEUE:
                    self._report_queue_overflow("DATA")
                    return

                self._data_q.append(raw)
                self._schedule_work()

    def _report_queue_overflow(self, queue_name):
        self._queue_overflow_reported = True

        self._notify_json({
            "op": "BUSY",
            "reason": "QUEUE_FULL",
            "queue": queue_name,
        })

    def _schedule_work(self):
        if not self._work_scheduled:
            self._work_scheduled = True

            try:
                micropython.schedule(self._work_cb, 0)
            except RuntimeError:
                self._work_scheduled = False
                self._report_queue_overflow("SCHEDULER")

    def _work_cb(self, _):
        self._work_scheduled = False
        self._queue_overflow_reported = False

        self._process_control_stream(limit_frags=80)
        self._process_data_frames(limit=120)

        if self._control_frag_q or self._data_q:
            self._schedule_work()

    # -------------------------------------------------------------------------
    # Notify and console output
    # -------------------------------------------------------------------------

    def _notify_json(self, obj):
        if self._conn_handle is None:
            return

        try:
            payload = ujson.dumps(obj).encode() + b"\n"
        except Exception:
            payload = b'{"op":"ERROR","code":"NOTIFY_SERIALIZE"}\n'

        sent = 0

        for i in range(0, len(payload), MTU_SIZE):
            chunk = payload[i:i + MTU_SIZE]

            try:
                self._ble.gatts_notify(
                    self._conn_handle,
                    self._h_notify,
                    chunk,
                )
                sent += 1
            except Exception:
                # Cannot safely notify an error here because notify itself failed.
                break

            utime.sleep_ms(NOTIFY_CHUNK_DELAY_MS)

            if sent % NOTIFY_YIELD_EVERY == 0:
                try:
                    gc.collect()
                except Exception:
                    pass

    def _error(self, code, msg=None, extra=None):
        o = {
            "op": "ERROR",
            "code": str(code),
        }

        if msg is not None:
            o["msg"] = _safe_str_preview(msg, 200)

        if extra is not None:
            o["extra"] = extra

        self._notify_json(o)

    def console_write(self, text, stream="stdout"):
        try:
            txt = str(text)
        except Exception:
            txt = "<console_write stringify failed>"

        for i in range(0, len(txt), MTU_SIZE):
            self._notify_json({
                "op": "CONSOLE",
                "stream": stream,
                "text": txt[i:i + MTU_SIZE],
            })

        return True

    # -------------------------------------------------------------------------
    # Control command parsing
    # -------------------------------------------------------------------------

    def _ctrl_idle_reset_if_needed(self):
        now = utime.ticks_ms()

        if (
            len(self._ctrl_buf)
            and utime.ticks_diff(now, self._ctrl_last_rx_ms) > self._CTRL_IDLE_RESET_MS
        ):
            self._ctrl_buf = bytearray()

    def _process_control_stream(self, limit_frags=20):
        for _ in range(limit_frags):
            if not self._control_frag_q:
                return

            self._ctrl_idle_reset_if_needed()

            frag = self._control_frag_q.pop(0)
            self._ctrl_last_rx_ms = utime.ticks_ms()

            if len(self._ctrl_buf) + len(frag) > self._CTRL_MAX_BUF:
                self._error("CTRL_BUF_OVERFLOW", extra={
                    "buf_len": len(self._ctrl_buf),
                    "frag_len": len(frag),
                })
                self._ctrl_buf = bytearray()
                return

            self._ctrl_buf.extend(frag)

            while True:
                nl = _ba_find(self._ctrl_buf, b"\n")

                if nl < 0:
                    break

                line = bytes(self._ctrl_buf[:nl])
                self._ctrl_buf = _ba_consume(self._ctrl_buf, nl + 1)
                self._handle_control_line(line)

    def _handle_control_line(self, line_bytes):
        try:
            cleaned = line_bytes.decode("utf-8", "strict").replace("\x00", "").strip()
        except Exception as e:
            self._error("BAD_UTF8", str(e))
            return

        if not cleaned:
            return

        try:
            cmd = ujson.loads(cleaned)
        except Exception as e:
            self._error("BAD_JSON", str(e), {
                "raw_preview": _safe_str_preview(cleaned, 200),
            })
            return

        if not isinstance(cmd, dict):
            self._error("JSON_NOT_OBJECT")
            return

        self._handle_cmd(cmd)

    # -------------------------------------------------------------------------
    # DATA frames
    # -------------------------------------------------------------------------

    def _process_data_frames(self, limit=50):
        for _ in range(limit):
            if not self._data_q:
                return

            self._handle_data_frame(self._data_q.pop(0))

    def _handle_data_frame(self, frame):
        if not self._put_active or self._put_fp is None:
            self._notify_json({
                "op": "PUT_NACK",
                "seq": 0,
                "reason": "NOACTIVE",
            })
            return

        if len(frame) < DATA_HEADER_SIZE:
            self._notify_json({
                "op": "PUT_NACK",
                "seq": 0,
                "reason": "SHORT",
            })
            return

        seq = _u32le(frame, 0)
        offset = _u32le(frame, 4)
        payload = frame[DATA_HEADER_SIZE:]

        if self._put_crc32_chunks:
            if len(payload) < 4:
                self._notify_json({
                    "op": "PUT_NACK",
                    "seq": seq,
                    "reason": "NO_CRC",
                })
                return

            want = _u32le(payload, len(payload) - 4)
            body = payload[:-4]
            got = _crc32_u32(body)

            if got != want:
                self._notify_json({
                    "op": "PUT_NACK",
                    "seq": seq,
                    "reason": "CRC32",
                    "got": got,
                    "want": want,
                })
                return

            payload = body

        if offset + len(payload) > self._put_size:
            self._notify_json({
                "op": "PUT_NACK",
                "seq": seq,
                "reason": "BOUNDS",
            })
            return

        try:
            if offset != self._put_next_offset:
                self._put_fp.seek(offset)

            self._put_fp.write(payload)

            self._put_next_offset = offset + len(payload)
            self._put_received = max(self._put_received, self._put_next_offset)

            should_ack = (
                self._ack_every <= 1
                or (seq % self._ack_every) == 0
                or self._put_next_offset >= self._put_size
            )

            if should_ack:
                self._put_last_ack_seq = seq
                self._notify_json({
                    "op": "PUT_ACK",
                    "seq": seq,
                    "next_offset": self._put_next_offset,
                })

        except Exception as e:
            self._notify_json({
                "op": "PUT_NACK",
                "seq": seq,
                "reason": "WRITE",
                "msg": _safe_str_preview(e, 140),
            })

    # -------------------------------------------------------------------------
    # Commands
    # -------------------------------------------------------------------------

    def _handle_cmd(self, cmd):
        op = str(cmd.get("op", "")).strip().upper()

        if op == "HELLO":
            multi = _program_capabilities()
            self._notify_json({
                "op": "HELLO_ACK",
                "ver": 5,
                "device": self._name,
                "service": "EvoDownloadManager",
                "mode": (
                    "multiple_program_filesystem"
                    if multi["supported"]
                    else "root_main_py"
                ),
                "main_file": "/main.py",
                "multiple_program_filesystem": multi["supported"],
                "program_root": multi["root"],
                "program_main_file": multi["main_file"],
                "capabilities": {
                    "multiple_programs": multi,
                },
                "mtu_size": MTU_SIZE,
                "data_payload_size": DATA_PAYLOAD_SIZE,
                "ack_every": self._ack_every,
            })
            return

        if op == "INFO":
            multi = _program_capabilities()
            self._notify_json({
                "op": "INFO_RESULT",
                "mode": (
                    "multiple_program_filesystem"
                    if multi["supported"]
                    else "root_main_py"
                ),
                "main_file": "/main.py",
                "program_root": multi["root"],
                "program_main_file": multi["main_file"],
                "multiple_program_filesystem": multi["supported"],
                "capabilities": {
                    "multiple_programs": multi,
                },
                "mtu_size": MTU_SIZE,
                "data_payload_size": DATA_PAYLOAD_SIZE,
                "root_only": not multi["supported"],
                "list_root_only": False,
                "py_only": True,
                "run_supported": False,
                "reset_after_upload": True,
                "boot_upload_allowed": True,
                "sensor_read_contract": "read() -> dict",
                "supports": self._supports,
            })
            return

        if op == "LIST":
            path = _normalise_path(cmd.get("path", "/"))

            if not path:
                self._error("LIST_BAD_PATH", extra={
                    "path": cmd.get("path", "/"),
                })
                return

            self._notify_json({
                "op": "LIST_BEGIN",
                "path": path,
            })

            try:
                self._notify_json({
                    "op": "LIST_RESULT",
                    "path": path,
                    "items": _list_dir(path),
                })
            except Exception as e:
                self._error("LIST_FAIL", str(e), {
                    "path": path,
                })

            return

        if op == "CRC32":
            path = _normalise_path(cmd.get("path"))

            if not path:
                self._error("CRC32_ARGS")
                return

            if not (_is_root_file(path) or _is_program_file(path)):
                self._error("CRC32_ROOT_ONLY", extra={
                    "path": path,
                })
                return

            try:
                self._notify_json({
                    "op": "CRC32_RESULT",
                    "path": path,
                    "crc32": _file_crc32(path),
                })
            except Exception as e:
                self._error("CRC32_FAIL", str(e), {
                    "path": path,
                })

            return

        if op == "RESUME":
            path = _normalise_path(cmd.get("path"))

            ok, result = _validate_upload_path(path)

            if not ok:
                self._error("RESUME_" + result, extra={
                    "path": path,
                })
                return

            path = result
            tmp_path = _tmp_path_for(path)
            st = _stat_path(tmp_path)

            if not st:
                self._notify_json({
                    "op": "RESUME_RESULT",
                    "path": path,
                    "tmp_path": tmp_path,
                    "exists": False,
                    "size": 0,
                })
                return

            size = int(st[6]) if len(st) > 6 else 0

            self._notify_json({
                "op": "RESUME_RESULT",
                "path": path,
                "tmp_path": tmp_path,
                "exists": True,
                "size": size,
            })

            return

        if op == "ABORT":
            reason = cmd.get("reason", "CLIENT_ABORT")
            self._abort_put(reason, notify=True, remove_tmp=True)
            self._notify_json({
                "op": "ABORT_OK",
                "reason": reason,
            })
            return

        if op == "PUT_BEGIN":
            path = _normalise_path(cmd.get("path"))

            try:
                size = int(cmd.get("size", 0))
            except Exception:
                size = -1

            ok, result = _validate_upload_path(path)

            if not ok:
                self._error("PUT_" + result, extra={
                    "path": path,
                    "hint": (
                        "Use root .py files, or /programs/<name>/*.py when multiple_program_filesystem is true"
                    ),
                })
                return

            if size < 0:
                self._error("PUT_BEGIN_ARGS", extra={
                    "path": path,
                    "size": size,
                })
                return

            path = result
            tmp_path = _tmp_path_for(path)

            crc32_file = cmd.get("crc32", None)

            try:
                self._put_expected_crc32 = (
                    int(crc32_file) if crc32_file is not None else None
                )
            except Exception:
                self._put_expected_crc32 = None

            self._put_crc32_chunks = bool(cmd.get("crc32_chunks", False))

            try:
                self._abort_put("NEW_PUT", notify=False, remove_tmp=False)

                resume = bool(cmd.get("resume", False))
                offset = 0
                mode = "wb"

                if resume:
                    st = _stat_path(tmp_path)

                    if st and len(st) > 6:
                        offset = int(st[6])

                        if offset > size:
                            offset = 0

                        mode = "ab" if offset > 0 else "wb"

                _ensure_parent_dirs(tmp_path)
                self._put_fp = open(tmp_path, mode)
                self._put_active = True
                self._put_path = path
                self._put_tmp_path = tmp_path
                self._put_size = size
                self._put_next_offset = offset
                self._put_received = offset
                self._put_last_ack_seq = -1

                self._notify_json({
                    "op": "PUT_READY",
                    "path": path,
                    "tmp_path": tmp_path,
                    "offset": offset,
                    "data_payload_size": DATA_PAYLOAD_SIZE,
                    "mtu_size": MTU_SIZE,
                    "crc32_chunks": self._put_crc32_chunks,
                    "safe_write": True,
                    "root_only": not _programs_enabled(),
                    "py_only": True,
                    "multiple_program_filesystem": _programs_enabled(),
                })

            except Exception as e:
                self._abort_put("PUT_OPEN_FAIL", notify=False, remove_tmp=False)
                self._error("PUT_OPEN_FAIL", str(e), {
                    "path": path,
                    "tmp_path": tmp_path,
                })

            return

        if op == "PUT_END":
            path = _normalise_path(cmd.get("path", self._put_path))

            if (
                not self._put_active
                or self._put_fp is None
                or path != self._put_path
            ):
                self._error("PUT_END_NOACTIVE", extra={
                    "path": path,
                    "active": self._put_active,
                })
                return

            try:
                try:
                    self._put_fp.flush()
                except Exception:
                    pass

                try:
                    os.sync()
                except Exception:
                    pass

                self._put_fp.close()
                self._put_fp = None

                if self._put_received != self._put_size:
                    self._error("PUT_SIZE_MISMATCH", extra={
                        "path": self._put_path,
                        "received": self._put_received,
                        "expected": self._put_size,
                    })
                    self._abort_put("SIZE_MISMATCH", notify=True, remove_tmp=False)
                    return

                if self._put_expected_crc32 is not None:
                    got = _file_crc32(self._put_tmp_path)
                    want = self._put_expected_crc32 & 0xFFFFFFFF

                    if got != want:
                        self._error("PUT_CRC_MISMATCH", extra={
                            "path": self._put_path,
                            "got": got,
                            "want": want,
                        })
                        self._abort_put("CRC_MISMATCH", notify=True, remove_tmp=True)
                        return

                final_path = self._put_path
                tmp_path = self._put_tmp_path
                final_size = self._put_received

                _safe_replace(tmp_path, final_path)
                _mark_program_download(final_path)

                self._put_active = False
                self._put_path = None
                self._put_tmp_path = None
                self._put_size = 0
                self._put_next_offset = 0
                self._put_received = 0
                self._put_expected_crc32 = None
                self._put_crc32_chunks = False

                self._notify_json({
                    "op": "PUT_OK",
                    "path": final_path,
                    "size": final_size,
                    "safe_write": True,
                    "reset_required": True,
                    "is_main": final_path == "/main.py",
                    "is_boot": final_path == "/boot.py",
                })

            except Exception as e:
                self._abort_put("PUT_END_FAIL", notify=True, remove_tmp=False)
                self._error("PUT_END_FAIL", str(e), {
                    "path": path,
                })

            return

        if op == "RESET":
            self._sensor_stream_stop({})
            # A soft reset restarts MicroPython but can leave external I2C
            # peripherals, such as the motor PCA9685, in a stale state after a
            # downloaded program has run. Default to a full board reset after
            # uploads so each run starts like the first run after flashing.
            mode = cmd.get("mode", "hard")
            self._notify_json({
                "op": "RESETTING",
                "mode": mode,
            })
            utime.sleep_ms(80)

            if mode == "hard":
                machine.reset()
            else:
                machine.soft_reset()

            return

        if op == "SENSOR_LIST":
            self._sensor_list()
            return

        if op == "SENSOR_READ":
            self._sensor_read_once(cmd)
            return

        if op == "SENSOR_STREAM_START":
            self._sensor_stream_start(cmd)
            return

        if op == "SENSOR_STREAM_STOP":
            self._sensor_stream_stop(cmd)
            return

        if op == "RUN":
            self._error("RUN_DISABLED", extra={
                "hint": "Upload /main.py and send RESET instead",
            })
            return

        self._error("UNKNOWN_OP", extra={
            "op": op,
        })

    # -------------------------------------------------------------------------
    # PUT abort
    # -------------------------------------------------------------------------

    def _abort_put(self, reason, notify=True, remove_tmp=False):
        if self._put_fp:
            try:
                self._put_fp.close()
            except Exception:
                pass

        was_active = self._put_active
        old_path = self._put_path
        old_tmp = self._put_tmp_path

        if remove_tmp and old_tmp:
            try:
                os.remove(old_tmp)
            except Exception:
                pass

        self._put_fp = None
        self._put_active = False
        self._put_path = None
        self._put_tmp_path = None
        self._put_size = 0
        self._put_next_offset = 0
        self._put_received = 0
        self._put_expected_crc32 = None
        self._put_crc32_chunks = False

        if notify and was_active:
            self._notify_json({
                "op": "PUT_ABORTED",
                "reason": str(reason),
                "path": old_path,
                "tmp_path": old_tmp,
                "tmp_removed": bool(remove_tmp),
            })

    # -------------------------------------------------------------------------
    # Sensor commands
    # -------------------------------------------------------------------------

    def _sensor_list(self):
        sensors = []

        for name, pair in _SENSOR_MODULES.items():
            module_name, class_name = pair
            available = False
            error = None

            try:
                cls = _import_attr(module_name, class_name)
                available = hasattr(cls, "__call__")
            except Exception as e:
                error = _safe_str_preview(e, 80)

            sensors.append({
                "name": name,
                "module": module_name,
                "class": class_name,
                "available": available,
                "read_contract": "read() -> dict",
                "error": error,
            })

        self._notify_json({
            "op": "SENSOR_LIST_RESULT",
            "sensors": sensors,
        })

    def _sensor_read_once(self, cmd):
        if not self._sensor_streaming_enabled:
            self._error("SENSOR_DISABLED")
            return

        sensor = _normalise_sensor_name(cmd.get("sensor"))
        port = cmd.get("port", None)
        options = cmd.get("options", None)

        stream_id = cmd.get(
            "stream_id",
            sensor + (":" + str(port) if port is not None else ""),
        )

        try:
            obj = _construct_sensor(sensor, port, options)
            data = _read_sensor(sensor, obj)

            self._notify_json({
                "op": "SENSOR_DATA",
                "mode": "read",
                "stream_id": stream_id,
                "sensor": sensor,
                "port": port,
                "time_ms": utime.ticks_ms(),
                "data": data,
            })

        except Exception as e:
            self._error("SENSOR_READ_FAIL", str(e), {
                "sensor": sensor,
                "port": port,
            })

    def _sensor_stream_start(self, cmd):
        if not self._sensor_streaming_enabled:
            self._error("SENSOR_DISABLED")
            return

        sensor = _normalise_sensor_name(cmd.get("sensor"))
        port = cmd.get("port", None)
        options = cmd.get("options", None)

        if not sensor:
            self._error("SENSOR_STREAM_ARGS")
            return

        try:
            interval_ms = max(20, int(cmd.get("interval_ms", 100)))
        except Exception:
            interval_ms = 100

        stream_id = cmd.get(
            "stream_id",
            sensor + (":" + str(port) if port is not None else ""),
        )

        try:
            obj = _construct_sensor(sensor, port, options)

            # Validate sensor contract immediately.
            test_data = _read_sensor(sensor, obj)

            self._sensor_streams[stream_id] = {
                "sensor": sensor,
                "port": port,
                "obj": obj,
                "interval_ms": interval_ms,
                "next_ms": utime.ticks_add(utime.ticks_ms(), interval_ms),
                "seq": 0,
                "errors": 0,
            }

            self._ensure_sensor_timer()

            self._notify_json({
                "op": "SENSOR_STREAM_STARTED",
                "stream_id": stream_id,
                "sensor": sensor,
                "port": port,
                "interval_ms": interval_ms,
                "first_data": test_data,
            })

        except Exception as e:
            self._error("SENSOR_STREAM_START_FAIL", str(e), {
                "sensor": sensor,
                "port": port,
            })

    def _sensor_stream_stop(self, cmd=None):
        cmd = cmd or {}
        stream_id = cmd.get("stream_id", None)

        if stream_id:
            existed = stream_id in self._sensor_streams

            try:
                del self._sensor_streams[stream_id]
            except Exception:
                pass

            self._notify_json({
                "op": "SENSOR_STREAM_STOPPED",
                "stream_id": stream_id,
                "existed": existed,
            })

        else:
            count = len(self._sensor_streams)
            self._sensor_streams = {}

            self._notify_json({
                "op": "SENSOR_STREAM_STOPPED",
                "all": True,
                "count": count,
            })

        if not self._sensor_streams:
            self._stop_sensor_timer()

    def _ensure_sensor_timer(self):
        if self._sensor_timer is not None:
            return

        try:
            self._sensor_timer = machine.Timer(-1)
            self._sensor_timer.init(
                period=self._sensor_tick_ms,
                mode=machine.Timer.PERIODIC,
                callback=self._sensor_timer_cb,
            )

        except Exception as e:
            self._sensor_timer = None
            self._error("SENSOR_TIMER_FAIL", str(e))

    def _stop_sensor_timer(self):
        if self._sensor_timer is not None:
            try:
                self._sensor_timer.deinit()
            except Exception:
                pass

            self._sensor_timer = None

        self._sensor_work_scheduled = False

    def _sensor_timer_cb(self, timer):
        if self._sensor_work_scheduled:
            return

        self._sensor_work_scheduled = True

        try:
            micropython.schedule(self._sensor_work_cb, 0)
        except RuntimeError:
            self._sensor_work_scheduled = False

    def _sensor_work_cb(self, _):
        self._sensor_work_scheduled = False

        if not self._sensor_streams or self._conn_handle is None:
            return

        now = utime.ticks_ms()

        for stream_id in list(self._sensor_streams.keys()):
            st = self._sensor_streams.get(stream_id)

            if not st:
                continue

            if utime.ticks_diff(now, st["next_ms"]) < 0:
                continue

            st["next_ms"] = utime.ticks_add(now, st["interval_ms"])

            try:
                data = _read_sensor(st["sensor"], st["obj"])

                st["seq"] += 1
                st["errors"] = 0

                self._notify_json({
                    "op": "SENSOR_DATA",
                    "mode": "stream",
                    "stream_id": stream_id,
                    "sensor": st["sensor"],
                    "port": st["port"],
                    "seq": st["seq"],
                    "time_ms": now,
                    "data": data,
                })

            except Exception as e:
                st["errors"] += 1

                self._error("SENSOR_STREAM_READ_FAIL", str(e), {
                    "stream_id": stream_id,
                    "sensor": st["sensor"],
                    "port": st["port"],
                    "errors": st["errors"],
                })

                if st["errors"] >= 5:
                    try:
                        del self._sensor_streams[stream_id]
                    except Exception:
                        pass

                    self._notify_json({
                        "op": "SENSOR_STREAM_STOPPED",
                        "stream_id": stream_id,
                        "reason": "READ_FAIL",
                    })

        if not self._sensor_streams:
            self._stop_sensor_timer()


# ============================================================================
# Public API
# ============================================================================

def start(
    name=None,
    adv_interval_us=None,
    debug=None,
    ack_every=None,
    sensor_streaming=None,
    sensor_tick_ms=None,
):
    global _service_instance

    cfg = _cfg_load()

    if name is None:
        name = cfg.get("name", _DEFAULT_ROOT_CFG["name"])

    if adv_interval_us is None:
        adv_interval_us = int(cfg.get(
            "adv_interval_us",
            _DEFAULT_BLE_DOWNLOAD_CFG["adv_interval_us"],
        ))

    if debug is None:
        debug = bool(cfg.get("debug", _DEFAULT_BLE_DOWNLOAD_CFG["debug"]))

    if ack_every is None:
        ack_every = int(cfg.get("ack_every", _DEFAULT_BLE_DOWNLOAD_CFG["ack_every"]))

    if sensor_streaming is None:
        sensor_streaming = bool(cfg.get(
            "sensor_streaming",
            _DEFAULT_BLE_DOWNLOAD_CFG["sensor_streaming"],
        ))

    if sensor_tick_ms is None:
        sensor_tick_ms = int(cfg.get(
            "sensor_tick_ms",
            _DEFAULT_BLE_DOWNLOAD_CFG["sensor_tick_ms"],
        ))

    if _service_instance is None:
        _service_instance = EvoDownloadManager(
            name=name,
            debug=debug,
        )
    else:
        _service_instance._debug = bool(debug)
        _service_instance.set_name(name)

    _service_instance._ack_every = max(1, int(ack_every))
    _service_instance._sensor_streaming_enabled = bool(sensor_streaming)
    _service_instance._sensor_tick_ms = max(20, int(sensor_tick_ms))
    _service_instance._adv_interval_us = int(adv_interval_us)

    _service_instance.start_advertising(interval_us=adv_interval_us)

    return _service_instance


def stop():
    if _service_instance is None:
        return

    try:
        _service_instance._sensor_stream_stop({})
    except Exception:
        pass

    try:
        if _service_instance._conn_handle is not None:
            _service_instance._ble.gap_disconnect(_service_instance._conn_handle)
    except Exception:
        pass

    try:
        _service_instance.stop_advertising()
    except Exception:
        pass


def auto_start():
    try:
        cfg = _cfg_load()

        if not cfg.get("bluetooth_enabled", True):
            return None

        if not cfg.get("start_on_boot", True):
            return None

        return start()

    except Exception as e:
        try:
            print("EvoDownloadManager failed:", e)
        except Exception:
            pass

        return None


def console_write(text, stream="stdout"):
    if _service_instance is None:
        return False

    return _service_instance.console_write(text, stream=stream)


def console_print(*args, sep=" ", end="\n", stream="stdout"):
    try:
        s = sep.join([str(x) for x in args]) + end
    except Exception:
        s = "<console_print stringify failed>" + end

    return console_write(s, stream=stream)


def status():
    cfg = _cfg_load()
    multi = _program_capabilities()

    return {
        "bluetooth_enabled": cfg.get("bluetooth_enabled", True),
        "start_on_boot": cfg.get("start_on_boot", True),
        "name": cfg.get("name"),
        "controller_type": cfg.get("controller_type"),
        "multiple_program_filesystem": multi["supported"],
        "capabilities": {
            "multiple_programs": multi,
        },
        "adv_interval_us": cfg.get("adv_interval_us"),
        "mtu_size": MTU_SIZE,
        "data_payload_size": DATA_PAYLOAD_SIZE,
        "ack_every": cfg.get("ack_every", 1),
        "sensor_streaming": cfg.get("sensor_streaming", True),
        "sensor_tick_ms": cfg.get("sensor_tick_ms", 50),
        "sensor_stream_count": (
            len(_service_instance._sensor_streams)
            if _service_instance is not None
            else 0
        ),
        "debug": cfg.get("debug", False),
        "running": _service_instance is not None,
        "connected": (
            _service_instance is not None
            and _service_instance._conn_handle is not None
        ),
        "service": "EvoDownloadManager",
        "mode": (
            "multiple_program_filesystem"
            if multi["supported"]
            else "root_main_py"
        ),
        "console_mode": "always_on_explicit_write",
        "commands": [
            "HELLO",
            "INFO",
            "LIST",
            "CRC32",
            "RESUME",
            "PUT_BEGIN",
            "PUT_END",
            "ABORT",
            "RESET",
            "SENSOR_LIST",
            "SENSOR_READ",
            "SENSOR_STREAM_START",
            "SENSOR_STREAM_STOP",
        ],
    }


def cli(cmd=None):
    if cmd is None:
        return status()

    c = str(cmd).strip()

    if not c:
        return status()

    parts = c.split()
    head = parts[0].lower()

    if head in ("status", "st"):
        return status()

    if head in ("on", "enable"):
        enable_persistent(True)
        try:
            start()
        except Exception:
            pass
        return status()

    if head in ("off", "disable"):
        enable_persistent(False)
        try:
            stop()
        except Exception:
            pass
        return status()

    if head == "name":
        if len(parts) >= 2:
            set_name_persistent(" ".join(parts[1:]))
        return status()

    if head == "ack":
        if len(parts) >= 2:
            set_ack_every_persistent(int(parts[1]))
        return status()

    if head in ("boot", "start_on_boot"):
        if len(parts) >= 2 and parts[1].lower() in ("on", "enable", "1", "true"):
            start_on_boot_persistent(True)
        elif len(parts) >= 2 and parts[1].lower() in ("off", "disable", "0", "false"):
            start_on_boot_persistent(False)
        return status()

    if head == "sensor":
        if len(parts) >= 2 and parts[1].lower() in ("on", "enable", "1", "true"):
            sensor_streaming_persistent(True)
        elif len(parts) >= 2 and parts[1].lower() in ("off", "disable", "0", "false"):
            sensor_streaming_persistent(False)
        return status()

    if head == "start":
        enable_persistent(True)
        try:
            start()
        except Exception:
            pass
        return status()

    if head == "stop":
        try:
            stop()
        except Exception:
            pass
        return status()

    return {
        "error": "unknown command",
        "cmd": c,
        "hint": "status|on|off|boot on/off|name <x>|ack <n>|sensor on/off|start|stop",
    }
