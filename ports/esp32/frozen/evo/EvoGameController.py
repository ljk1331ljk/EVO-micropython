# EvoGameController.py
# Frozen MicroPython BLE HID client for GamePadPlus V3 / similar HID gamepads
#
# Stable version:
# - no sleep inside IRQ
# - no connect inside IRQ
# - no user callbacks inside IRQ
# - IRQ only stores state / flags
# - user calls update() regularly to drive scan/connect/discovery state machine
#
# Verified report format for your controller:
#   d[0]=LX, d[1]=LY, d[2]=RX, d[3]=RY
#   d[4]=hat low nibble, 0x0F neutral
#   d[5]=buttons low, d[6]=buttons high
#   d[7]=LT, d[8]=RT, d[9]=reserved

import bluetooth
import struct
import time
from micropython import const

# IRQs
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_DONE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_SERVICE_DONE = const(10)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_CHARACTERISTIC_DONE = const(12)
_IRQ_GATTC_DESCRIPTOR_RESULT = const(13)
_IRQ_GATTC_DESCRIPTOR_DONE = const(14)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)

# UUIDs
_UUID_HID_SERVICE = bluetooth.UUID(0x1812)
_UUID_PROTOCOL_MODE = bluetooth.UUID(0x2A4E)
_UUID_REPORT = bluetooth.UUID(0x2A4D)
_UUID_CCCD = bluetooth.UUID(0x2902)

# Name match modes
_NAME_MATCH_ANY = const(0)
_NAME_MATCH_CONTAINS = const(1)
_NAME_MATCH_EXACT = const(2)

# Internal phases
_PHASE_IDLE = const(0)
_PHASE_SCANNING = const(1)
_PHASE_CONNECTING = const(2)
_PHASE_DISCOVER_SERVICES = const(3)
_PHASE_DISCOVER_CHARS = const(4)
_PHASE_DISCOVER_DESCS = const(5)
_PHASE_WRITE_PROTO = const(6)
_PHASE_WRITE_CCCD = const(7)
_PHASE_READY = const(8)
_PHASE_FAILED = const(9)

# Button bit masks
BTN_A = const(0x0001)
BTN_B = const(0x0002)
BTN_X = const(0x0008)
BTN_Y = const(0x0010)

BTN_L1 = const(0x0040)
BTN_R1 = const(0x0080)
BTN_L2 = const(0x0100)
BTN_R2 = const(0x0200)

BTN_SELECT = const(0x0400)
BTN_START = const(0x0800)

# D-pad exposed as button bits too
BTN_UP = const(0x10000)
BTN_RIGHT = const(0x20000)
BTN_DOWN = const(0x40000)
BTN_LEFT = const(0x80000)

# Hat values
HAT_NEUTRAL = const(-1)
HAT_UP = const(0)
HAT_RIGHT = const(2)
HAT_DOWN = const(4)
HAT_LEFT = const(6)


def _decode_name(adv_data):
    i = 0
    n = len(adv_data)
    while i + 1 < n:
        l = adv_data[i]
        if l == 0:
            break
        t = adv_data[i + 1]
        if t in (0x08, 0x09):
            try:
                return adv_data[i + 2:i + 1 + l].decode()
            except Exception:
                return ""
        i += 1 + l
    return ""


def _adv_has_uuid16(adv_data, uuid16):
    i = 0
    n = len(adv_data)
    while i + 1 < n:
        l = adv_data[i]
        if l == 0:
            break
        t = adv_data[i + 1]
        if t in (0x02, 0x03):
            field = adv_data[i + 2:i + 1 + l]
            for j in range(0, len(field), 2):
                if j + 2 <= len(field):
                    u = struct.unpack("<H", field[j:j + 2])[0]
                    if u == uuid16:
                        return True
        i += 1 + l
    return False


def _clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _apply_deadzone(x, dz):
    if dz <= 0:
        return x
    ax = abs(x)
    if ax < dz:
        return 0.0
    y = (ax - dz) / (1.0 - dz)
    if y > 1.0:
        y = 1.0
    return y if x >= 0 else -y


def _norm_u8_01(v):
    return v / 255.0


def _norm_center_fixed(v, c):
    if v == c:
        return 0.0
    if v > c:
        denom = 255 - c
        if denom < 1:
            denom = 1
        return (v - c) / denom
    else:
        denom = c
        if denom < 1:
            denom = 1
        return -((c - v) / denom)


class EvoGameController:
    def __init__(self, ble=None, debug=False):
        self._ble = ble or bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)

        self._debug = debug
        self.deadzone = 0.08

        self._name_pattern = ""
        self._name_match = _NAME_MATCH_ANY

        self._use_fixed_address = False
        self._fixed_addr_type = None
        self._fixed_addr = None

        self._reset_runtime()

    def _reset_runtime(self):
        self._conn_handle = None
        self._connected = False

        self._phase = _PHASE_IDLE
        self._op_deadline = 0
        self._last_error = None

        self._scan_done = False
        self._scan_found = False
        self._best_rssi = -999
        self._best_addr_type = None
        self._best_addr = None
        self._best_name = ""

        self._svc_start = None
        self._svc_end = None
        self._proto_value_handle = None
        self._report_value_handle = None
        self._report_cccd_handle = None

        self._services_done = False
        self._chars_done = False
        self._descs_done = False
        self._write_done = False

        self._raw = b""
        self._last_report_len = 0

        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.lt = 0.0
        self.rt = 0.0
        self.hat = HAT_NEUTRAL
        self.buttons = 0

        self._center_valid = False
        self._did_auto_center = False
        self._cx = 0x80
        self._cy = 0x80
        self._crx = 0x80
        self._cry = 0x80

    def _log(self, *args):
        if self._debug:
            print("[EvoGameController]", *args)

    def _set_deadline(self, ms):
        self._op_deadline = time.ticks_add(time.ticks_ms(), ms)

    def _expired(self):
        return time.ticks_diff(time.ticks_ms(), self._op_deadline) > 0

    def set_debug(self, enabled):
        self._debug = bool(enabled)

    def isConnected(self):
        return self._connected

    def latest_raw(self):
        return self._raw

    def isPressed(self, mask):
        return (self.buttons & mask) != 0

    def getLX(self):
        return self.lx

    def getLY(self):
        return self.ly

    def getRX(self):
        return self.rx

    def getRY(self):
        return self.ry

    def getLT(self):
        return self.lt

    def getRT(self):
        return self.rt

    def state(self):
        return {
            "connected": self._connected,
            "lx": self.lx,
            "ly": self.ly,
            "rx": self.rx,
            "ry": self.ry,
            "lt": self.lt,
            "rt": self.rt,
            "hat": self.hat,
            "buttons": self.buttons,
            "raw_len": self._last_report_len,
            "phase": self._phase,
            "error": self._last_error,
        }

    def begin(self, timeout_ms=5000):
        return self.begin_by_name("", _NAME_MATCH_ANY, timeout_ms=timeout_ms)

    def begin_by_name(self, name_pattern, match=_NAME_MATCH_CONTAINS, timeout_ms=5000):
        self._name_pattern = name_pattern or ""
        self._name_match = match
        self._use_fixed_address = False
        self._reset_runtime()

        self._scan_done = False
        self._scan_found = False
        self._best_rssi = -999
        self._best_addr_type = None
        self._best_addr = None
        self._best_name = ""

        self._log("Scanning...")
        self._ble.gap_scan(timeout_ms, 30000, 30000, True)
        self._phase = _PHASE_SCANNING
        self._set_deadline(timeout_ms + 1000)
        return True

    def begin_by_address(self, addr_type, addr, timeout_ms=5000):
        self._use_fixed_address = True
        self._fixed_addr_type = addr_type
        self._fixed_addr = bytes(addr)
        self._reset_runtime()

        self._log("Connecting by address...", addr_type, self._fixed_addr)
        try:
            self._ble.gap_connect(self._fixed_addr_type, self._fixed_addr)
        except Exception as e:
            self._last_error = "gap_connect:%s" % e
            self._phase = _PHASE_FAILED
            return False

        self._phase = _PHASE_CONNECTING
        self._set_deadline(timeout_ms)
        return True

    def disconnect(self):
        if self._conn_handle is not None:
            try:
                self._ble.gap_disconnect(self._conn_handle)
            except Exception:
                pass

    def update(self):
        if self._phase == _PHASE_IDLE or self._phase == _PHASE_READY:
            return self._connected

        if self._phase == _PHASE_SCANNING:
            if self._scan_found and self._best_addr is not None:
                try:
                    self._ble.gap_scan(None)
                except Exception:
                    pass
                self._log("Connecting to best scan result:", self._best_addr_type, self._best_addr, self._best_name)
                try:
                    self._ble.gap_connect(self._best_addr_type, self._best_addr)
                except Exception as e:
                    self._last_error = "gap_connect:%s" % e
                    self._phase = _PHASE_FAILED
                    return False
                self._phase = _PHASE_CONNECTING
                self._set_deadline(5000)
                return False

            if self._scan_done or self._expired():
                self._last_error = "scan_timeout_or_done"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_CONNECTING:
            if self._conn_handle is not None:
                self._ble.gattc_discover_services(self._conn_handle)
                self._phase = _PHASE_DISCOVER_SERVICES
                self._set_deadline(5000)
                return False

            if self._expired():
                self._last_error = "connect_timeout"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_DISCOVER_SERVICES:
            if self._services_done:
                if self._svc_start is None:
                    self._last_error = "hid_service_not_found"
                    self._phase = _PHASE_FAILED
                    return False
                self._ble.gattc_discover_characteristics(self._conn_handle, self._svc_start, self._svc_end)
                self._phase = _PHASE_DISCOVER_CHARS
                self._set_deadline(5000)
                return False

            if self._expired():
                self._last_error = "service_discovery_timeout"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_DISCOVER_CHARS:
            if self._chars_done:
                if self._report_value_handle is None:
                    self._last_error = "report_char_not_found"
                    self._phase = _PHASE_FAILED
                    return False
                self._ble.gattc_discover_descriptors(self._conn_handle, self._svc_start, self._svc_end)
                self._phase = _PHASE_DISCOVER_DESCS
                self._set_deadline(5000)
                return False

            if self._expired():
                self._last_error = "char_discovery_timeout"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_DISCOVER_DESCS:
            if self._descs_done:
                if self._proto_value_handle is not None:
                    self._write_done = False
                    try:
                        self._ble.gattc_write(self._conn_handle, self._proto_value_handle, b"\x01", 1)
                    except Exception as e:
                        self._last_error = "proto_write:%s" % e
                        self._phase = _PHASE_FAILED
                        return False
                    self._phase = _PHASE_WRITE_PROTO
                    self._set_deadline(2000)
                    return False

                if self._report_cccd_handle is None:
                    self._last_error = "cccd_not_found"
                    self._phase = _PHASE_FAILED
                    return False

                self._write_done = False
                try:
                    self._ble.gattc_write(self._conn_handle, self._report_cccd_handle, b"\x01\x00", 1)
                except Exception as e:
                    self._last_error = "cccd_write:%s" % e
                    self._phase = _PHASE_FAILED
                    return False
                self._phase = _PHASE_WRITE_CCCD
                self._set_deadline(2000)
                return False

            if self._expired():
                self._last_error = "desc_discovery_timeout"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_WRITE_PROTO:
            if self._write_done:
                if self._report_cccd_handle is None:
                    self._last_error = "cccd_not_found"
                    self._phase = _PHASE_FAILED
                    return False
                self._write_done = False
                try:
                    self._ble.gattc_write(self._conn_handle, self._report_cccd_handle, b"\x01\x00", 1)
                except Exception as e:
                    self._last_error = "cccd_write:%s" % e
                    self._phase = _PHASE_FAILED
                    return False
                self._phase = _PHASE_WRITE_CCCD
                self._set_deadline(2000)
                return False

            if self._expired():
                if self._report_cccd_handle is not None:
                    self._write_done = False
                    try:
                        self._ble.gattc_write(self._conn_handle, self._report_cccd_handle, b"\x01\x00", 1)
                    except Exception as e:
                        self._last_error = "cccd_write:%s" % e
                        self._phase = _PHASE_FAILED
                        return False
                    self._phase = _PHASE_WRITE_CCCD
                    self._set_deadline(2000)
                    return False
                self._last_error = "proto_write_timeout"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_WRITE_CCCD:
            if self._write_done:
                self._connected = True
                self._phase = _PHASE_READY
                self._log("Ready")
                return True

            if self._expired():
                self._last_error = "cccd_write_timeout"
                self._phase = _PHASE_FAILED
                return False

        elif self._phase == _PHASE_FAILED:
            return False

        return self._connected

    def _name_matches(self, adv_name):
        if self._name_match == _NAME_MATCH_ANY:
            return True
        if not self._name_pattern:
            return True
        if not adv_name:
            return False
        if self._name_match == _NAME_MATCH_EXACT:
            return adv_name == self._name_pattern
        return self._name_pattern in adv_name

    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data

            if not _adv_has_uuid16(adv_data, 0x1812):
                return

            name = _decode_name(adv_data)
            if not self._name_matches(name):
                return

            self._scan_found = True
            if rssi > self._best_rssi:
                self._best_rssi = rssi
                self._best_addr_type = addr_type
                self._best_addr = bytes(addr)
                self._best_name = name

        elif event == _IRQ_SCAN_DONE:
            self._scan_done = True

        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, addr_type, addr = data
            self._conn_handle = conn_handle
            self._connected = True
            self._log("Connected:", conn_handle, addr_type, addr)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            conn_handle, addr_type, addr = data
            if self._conn_handle == conn_handle:
                self._log("Disconnected:", conn_handle)
                self._conn_handle = None
                self._connected = False
                self._phase = _PHASE_IDLE
                self._did_auto_center = False
                self._center_valid = False

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            conn_handle, start_handle, end_handle, uuid = data
            if conn_handle == self._conn_handle and uuid == _UUID_HID_SERVICE:
                self._svc_start = start_handle
                self._svc_end = end_handle

        elif event == _IRQ_GATTC_SERVICE_DONE:
            conn_handle, status = data
            if conn_handle == self._conn_handle:
                self._services_done = True

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            conn_handle, def_handle, value_handle, properties, uuid = data
            if conn_handle != self._conn_handle:
                return

            if uuid == _UUID_REPORT and self._report_value_handle is None:
                self._report_value_handle = value_handle
            elif uuid == _UUID_PROTOCOL_MODE and self._proto_value_handle is None:
                self._proto_value_handle = value_handle

        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            conn_handle, status = data
            if conn_handle == self._conn_handle:
                self._chars_done = True

        elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
            conn_handle, dsc_handle, uuid = data
            if conn_handle != self._conn_handle:
                return
            if uuid == _UUID_CCCD and self._report_cccd_handle is None:
                self._report_cccd_handle = dsc_handle

        elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
            conn_handle, status = data
            if conn_handle == self._conn_handle:
                self._descs_done = True

        elif event == _IRQ_GATTC_WRITE_DONE:
            conn_handle, value_handle, status = data
            if conn_handle == self._conn_handle:
                self._write_done = True

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            if conn_handle != self._conn_handle:
                return

            self._raw = bytes(notify_data)
            self._last_report_len = len(self._raw)

            if not self._did_auto_center and len(self._raw) == 10:
                self._calibrate_center_from_raw10(self._raw)

            if len(self._raw) == 10:
                self._decode_gamepadplus_v3_10b(self._raw)

    def _calibrate_center_from_raw10(self, d):
        self._cx = d[0]
        self._cy = d[1]
        self._crx = d[2]
        self._cry = d[3]
        self._center_valid = True
        self._did_auto_center = True
        self._log("Auto-center:", self._cx, self._cy, self._crx, self._cry)

    def calibrate(self):
        if self._raw and len(self._raw) == 10:
            self._calibrate_center_from_raw10(self._raw)
            return True
        return False

    def _decode_gamepadplus_v3_10b(self, d):
        cx = self._cx if self._center_valid else 0x80
        cy = self._cy if self._center_valid else 0x80
        crx = self._crx if self._center_valid else 0x80
        cry = self._cry if self._center_valid else 0x80

        lx = _norm_center_fixed(d[0], cx)
        ly = _norm_center_fixed(d[1], cy)
        rx = _norm_center_fixed(d[2], crx)
        ry = _norm_center_fixed(d[3], cry)

        self.lx = _apply_deadzone(_clamp(lx, -1.0, 1.0), self.deadzone)
        self.ly = _apply_deadzone(_clamp(-ly, -1.0, 1.0), self.deadzone)
        self.rx = _apply_deadzone(_clamp(rx, -1.0, 1.0), self.deadzone)
        self.ry = _apply_deadzone(_clamp(-ry, -1.0, 1.0), self.deadzone)

        hat_nib = d[4] & 0x0F
        self.hat = HAT_NEUTRAL if hat_nib == 0x0F else hat_nib

        btn = d[5] | (d[6] << 8)

        # Map dpad hat into button bits too, so isPressed() can be used
        if self.hat == HAT_UP:
            btn |= BTN_UP
        elif self.hat == HAT_RIGHT:
            btn |= BTN_RIGHT
        elif self.hat == HAT_DOWN:
            btn |= BTN_DOWN
        elif self.hat == HAT_LEFT:
            btn |= BTN_LEFT

        self.buttons = btn
        self.lt = _norm_u8_01(d[7])
        self.rt = _norm_u8_01(d[8])