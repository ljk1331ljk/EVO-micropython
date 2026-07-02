#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/nlr.h"
#include "py/mphal.h"
#include "genhdr/mpversion.h"

#include <string.h>
#include <stdbool.h>

#include "driver/gpio.h"

#include "evo_pwm.h"
#include "evo_motor.h"
#include "evo_motorpair.h"
#include "evo_mecanum.h"

void evo_soft_reset_cleanup(void) {
    evo_pwm_clear_singleton();
}

// ============================================================================
// Shared Evo config
// ============================================================================

#define CONFIG_FILE "/evo_config.json"
#define CONFIG_TMP  "/evo_config.json.tmp"

#ifndef EVO_DEFAULT_NAME
#define EVO_DEFAULT_NAME "Evo"
#endif

#ifndef EVO_CONTROLLER_TYPE
#define EVO_CONTROLLER_TYPE "EVO"
#endif

#ifndef EVO_MULTIPLE_PROGRAM_FILESYSTEM
#define EVO_MULTIPLE_PROGRAM_FILESYSTEM (false)
#endif

#ifndef EVO_DOWNLOAD_ENABLED
#define EVO_DOWNLOAD_ENABLED (true)
#endif

#ifndef EVO_BLUETOOTH_ENABLED
#define EVO_BLUETOOTH_ENABLED EVO_DOWNLOAD_ENABLED
#endif

#ifndef EVO_DOWNLOAD_START_ON_BOOT
#define EVO_DOWNLOAD_START_ON_BOOT (true)
#endif

#ifndef EVO_DOWNLOAD_ADV_INTERVAL_US
#define EVO_DOWNLOAD_ADV_INTERVAL_US (200000)
#endif

#ifndef EVO_DOWNLOAD_DEBUG
#define EVO_DOWNLOAD_DEBUG (false)
#endif

#ifndef EVO_DOWNLOAD_ACK_EVERY
#define EVO_DOWNLOAD_ACK_EVERY (1)
#endif

#ifndef EVO_DOWNLOAD_SENSOR_STREAMING
#define EVO_DOWNLOAD_SENSOR_STREAMING (true)
#endif

#ifndef EVO_DOWNLOAD_SENSOR_TICK_MS
#define EVO_DOWNLOAD_SENSOR_TICK_MS (50)
#endif

#ifdef MICROPY_BUILD_TYPE
#define EVO_MICROPY_BUILD_TYPE_PAREN " (" MICROPY_BUILD_TYPE ")"
#else
#define EVO_MICROPY_BUILD_TYPE_PAREN
#endif

#ifndef EVO_DEFAULT_BLE_DOWNLOAD_ENABLED
#define EVO_DEFAULT_BLE_DOWNLOAD_ENABLED (false)
#endif

#ifndef EVO_DEFAULT_BLE_DOWNLOAD_ON_BOOT
#define EVO_DEFAULT_BLE_DOWNLOAD_ON_BOOT (false)
#endif

#define MAX_NAME_LEN 31
#define EVO_GPIO_STATE_COUNT GPIO_NUM_MAX

// ============================================================================
// Helpers
// ============================================================================

static mp_obj_t import_module(const char *name) {
    return mp_import_name(qstr_from_str(name), mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
}

static mp_obj_t new_str(const char *s) {
    return mp_obj_new_str(s, strlen(s));
}

static void file_close(mp_obj_t f) {
    mp_obj_t dest[2];
    mp_load_method(f, MP_QSTR_close, dest);
    mp_call_method_n_kw(0, 0, dest);
}

static bool is_allowed_char(char c) {
    return
        (c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') ||
        c == '_' || c == '-';
}

static bool is_valid_name(mp_obj_t obj) {
    if (!mp_obj_is_str(obj)) {
        return false;
    }

    size_t len = 0;
    const char *s = mp_obj_str_get_data(obj, &len);

    if (len == 0 || len > MAX_NAME_LEN) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        if (!is_allowed_char(s[i])) {
            return false;
        }
    }

    return true;
}

static mp_obj_t sanitize_name(mp_obj_t obj, const char *fallback) {
    if (!mp_obj_is_str(obj)) {
        return new_str(fallback);
    }

    size_t len = 0;
    const char *s = mp_obj_str_get_data(obj, &len);

    vstr_t vstr;
    vstr_init(&vstr, len);

    for (size_t i = 0; i < len && vstr.len < MAX_NAME_LEN; i++) {
        if (is_allowed_char(s[i])) {
            vstr_add_char(&vstr, s[i]);
        }
    }

    if (vstr.len == 0) {
        vstr_clear(&vstr);
        return new_str(fallback);
    }

    return mp_obj_new_str_from_vstr(&vstr);
}

static mp_obj_t dict_get_default(mp_obj_t dict, qstr key, mp_obj_t default_val) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t val = mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(key));
        nlr_pop();
        return val;
    }

    return default_val;
}

static bool obj_to_bool(mp_obj_t obj, bool fallback) {
    if (obj == mp_const_true) {
        return true;
    }
    if (obj == mp_const_false || obj == mp_const_none) {
        return false;
    }
    if (obj == MP_OBJ_NULL) {
        return fallback;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        bool v = mp_obj_is_true(obj);
        nlr_pop();
        return v;
    }

    return fallback;
}

static int obj_to_int(mp_obj_t obj, int fallback, int min_value) {
    if (obj == MP_OBJ_NULL || obj == mp_const_none) {
        return fallback;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        int v = mp_obj_get_int(obj);
        nlr_pop();
        if (v < min_value) {
            return min_value;
        }
        return v;
    }

    return fallback;
}

static gpio_num_t evo_get_gpio(mp_obj_t pin_obj) {
    int pin = mp_obj_get_int(pin_obj);
    if (pin < 0 || pin >= GPIO_NUM_MAX || pin >= 64) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid GPIO"));
    }
    return (gpio_num_t)pin;
}

static bool evo_gpio_is_pressed(gpio_num_t pin) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("gpio_config failed"));
    }

    return gpio_get_level(pin) == 0;
}

static bool evo_bump_last_pressed[EVO_GPIO_STATE_COUNT];

static mp_obj_t evo_is_pressed(mp_obj_t pin_obj) {
    gpio_num_t pin = evo_get_gpio(pin_obj);
    return mp_obj_new_bool(evo_gpio_is_pressed(pin));
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_is_pressed_obj, evo_is_pressed);

static mp_obj_t evo_is_released(mp_obj_t pin_obj) {
    gpio_num_t pin = evo_get_gpio(pin_obj);
    return mp_obj_new_bool(!evo_gpio_is_pressed(pin));
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_is_released_obj, evo_is_released);

static mp_obj_t evo_is_bumped(mp_obj_t pin_obj) {
    gpio_num_t pin = evo_get_gpio(pin_obj);
    bool pressed = evo_gpio_is_pressed(pin);
    bool bumped = pressed && !evo_bump_last_pressed[pin];
    evo_bump_last_pressed[pin] = pressed;
    return mp_obj_new_bool(bumped);
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_is_bumped_obj, evo_is_bumped);

static mp_obj_t default_download_config(void) {
    mp_obj_t d = mp_obj_new_dict(6);

    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_start_on_boot), mp_obj_new_bool(EVO_DOWNLOAD_START_ON_BOOT));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_adv_interval_us), mp_obj_new_int(EVO_DOWNLOAD_ADV_INTERVAL_US));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_debug), mp_obj_new_bool(EVO_DOWNLOAD_DEBUG));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_ack_every), mp_obj_new_int(EVO_DOWNLOAD_ACK_EVERY));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sensor_streaming), mp_obj_new_bool(EVO_DOWNLOAD_SENSOR_STREAMING));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sensor_tick_ms), mp_obj_new_int(EVO_DOWNLOAD_SENSOR_TICK_MS));

    return d;
}

static mp_obj_t default_config(void) {
    mp_obj_t d = mp_obj_new_dict(5);

    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_name), new_str(EVO_DEFAULT_NAME));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_controller_type), new_str(EVO_CONTROLLER_TYPE));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_multiple_program_filesystem), mp_obj_new_bool(EVO_MULTIPLE_PROGRAM_FILESYSTEM));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_bluetooth_enabled), mp_obj_new_bool(EVO_BLUETOOTH_ENABLED));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_download), default_download_config());

    return d;
}

static bool safe_write_config(mp_obj_t cfg) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t json = import_module("json");
        mp_obj_t os = import_module("os");
        mp_obj_t open_fn = mp_load_global(MP_QSTR_open);

        mp_obj_t f = mp_call_function_2(open_fn, new_str(CONFIG_TMP), new_str("w"));

        mp_obj_t dump = mp_load_attr(json, MP_QSTR_dump);
        mp_call_function_2(dump, cfg, f);

        file_close(f);

        nlr_buf_t nlr2;
        if (nlr_push(&nlr2) == 0) {
            mp_call_function_1(mp_load_attr(os, MP_QSTR_remove), new_str(CONFIG_FILE));
            nlr_pop();
        }

        mp_call_function_2(mp_load_attr(os, MP_QSTR_rename), new_str(CONFIG_TMP), new_str(CONFIG_FILE));

        nlr_pop();
        return true;
    }

    return false;
}

static void ensure_config(void) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t os = import_module("os");
        mp_call_function_1(mp_load_attr(os, MP_QSTR_stat), new_str(CONFIG_FILE));
        nlr_pop();
    } else {
        safe_write_config(default_config());
    }
}

static mp_obj_t sanitise_download_config(mp_obj_t raw_download) {
    mp_obj_t def = default_download_config();

    if (!mp_obj_is_type(raw_download, &mp_type_dict)) {
        return def;
    }

    mp_obj_t d = mp_obj_new_dict(6);

    mp_obj_t raw_start = dict_get_default(raw_download, MP_QSTR_start_on_boot,
        dict_get_default(def, MP_QSTR_start_on_boot, mp_const_true));
    mp_obj_t raw_adv = dict_get_default(raw_download, MP_QSTR_adv_interval_us,
        dict_get_default(def, MP_QSTR_adv_interval_us, mp_obj_new_int(200000)));
    mp_obj_t raw_debug = dict_get_default(raw_download, MP_QSTR_debug,
        dict_get_default(def, MP_QSTR_debug, mp_const_false));
    mp_obj_t raw_ack = dict_get_default(raw_download, MP_QSTR_ack_every,
        dict_get_default(def, MP_QSTR_ack_every, mp_obj_new_int(1)));
    mp_obj_t raw_sensor = dict_get_default(raw_download, MP_QSTR_sensor_streaming,
        dict_get_default(def, MP_QSTR_sensor_streaming, mp_const_true));
    mp_obj_t raw_tick = dict_get_default(raw_download, MP_QSTR_sensor_tick_ms,
        dict_get_default(def, MP_QSTR_sensor_tick_ms, mp_obj_new_int(50)));

    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_start_on_boot), mp_obj_new_bool(obj_to_bool(raw_start, EVO_DOWNLOAD_START_ON_BOOT)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_adv_interval_us), mp_obj_new_int(obj_to_int(raw_adv, EVO_DOWNLOAD_ADV_INTERVAL_US, 20000)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_debug), mp_obj_new_bool(obj_to_bool(raw_debug, EVO_DOWNLOAD_DEBUG)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_ack_every), mp_obj_new_int(obj_to_int(raw_ack, EVO_DOWNLOAD_ACK_EVERY, 1)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sensor_streaming), mp_obj_new_bool(obj_to_bool(raw_sensor, EVO_DOWNLOAD_SENSOR_STREAMING)));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_sensor_tick_ms), mp_obj_new_int(obj_to_int(raw_tick, EVO_DOWNLOAD_SENSOR_TICK_MS, 20)));

    return d;
}

static mp_obj_t load_config(void) {
    ensure_config();

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t json = import_module("json");
        mp_obj_t open_fn = mp_load_global(MP_QSTR_open);

        mp_obj_t f = mp_call_function_2(open_fn, new_str(CONFIG_FILE), new_str("r"));
        mp_obj_t load = mp_load_attr(json, MP_QSTR_load);
        mp_obj_t raw_cfg = mp_call_function_1(load, f);
        file_close(f);

        if (!mp_obj_is_type(raw_cfg, &mp_type_dict)) {
            mp_obj_t d = default_config();
            safe_write_config(d);
            nlr_pop();
            return d;
        }

        mp_obj_t raw_name = dict_get_default(raw_cfg, MP_QSTR_name, new_str(EVO_DEFAULT_NAME));
        mp_obj_t name = sanitize_name(raw_name, EVO_DEFAULT_NAME);

        mp_obj_t raw_ct = dict_get_default(raw_cfg, MP_QSTR_controller_type, new_str(EVO_CONTROLLER_TYPE));
        mp_obj_t controller_type;
        if (mp_obj_is_str(raw_ct)) {
            size_t ct_len = 0;
            mp_obj_str_get_data(raw_ct, &ct_len);
            controller_type = (ct_len == 0) ? new_str(EVO_CONTROLLER_TYPE) : raw_ct;
        } else {
            controller_type = new_str(EVO_CONTROLLER_TYPE);
        }

        mp_obj_t raw_multiple_program = dict_get_default(raw_cfg, MP_QSTR_multiple_program_filesystem,
            mp_obj_new_bool(EVO_MULTIPLE_PROGRAM_FILESYSTEM));
        mp_obj_t multiple_program = mp_obj_new_bool(obj_to_bool(raw_multiple_program, EVO_MULTIPLE_PROGRAM_FILESYSTEM));

        mp_obj_t raw_bluetooth_enabled = dict_get_default(raw_cfg, MP_QSTR_bluetooth_enabled,
            mp_obj_new_bool(EVO_BLUETOOTH_ENABLED));
        mp_obj_t bluetooth_enabled = mp_obj_new_bool(obj_to_bool(raw_bluetooth_enabled, EVO_BLUETOOTH_ENABLED));

        mp_obj_t raw_download = dict_get_default(raw_cfg, MP_QSTR_download, default_download_config());
        mp_obj_t download = sanitise_download_config(raw_download);

        mp_obj_t cfg = mp_obj_new_dict(5);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_name), name);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_controller_type), controller_type);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_multiple_program_filesystem), multiple_program);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_bluetooth_enabled), bluetooth_enabled);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_download), download);

        safe_write_config(cfg);

        nlr_pop();
        return cfg;
    }

    mp_obj_t d = default_config();
    safe_write_config(d);
    return d;
}

static mp_obj_t get_download_config(mp_obj_t cfg) {
    return dict_get_default(cfg, MP_QSTR_download, default_download_config());
}

static bool save_download_value(qstr key, mp_obj_t value) {
    mp_obj_t cfg = load_config();
    mp_obj_t download = get_download_config(cfg);
    mp_obj_dict_store(download, MP_OBJ_NEW_QSTR(key), value);
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_download), sanitise_download_config(download));
    return safe_write_config(cfg);
}

// ============================================================================
// evo_init(start_download=False, start_ble=False, debug=None, adv_interval_us=None,
//          ack_every=None, sensor_streaming=None, sensor_tick_ms=None)
//
// evo_init() always loads/creates /evo_config.json but only starts
// EvoDownloadManager when start_download=True or start_ble=True is explicitly
// passed.
// ============================================================================

static mp_obj_t evo_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    enum {
        ARG_start_download,
        ARG_start_ble,
        ARG_debug,
        ARG_adv_interval_us,
        ARG_ack_every,
        ARG_sensor_streaming,
        ARG_sensor_tick_ms,
        ARG_console,
        ARG_max_chunk,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_start_download, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_start_ble,      MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_debug,          MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_adv_interval_us, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_ack_every,      MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sensor_streaming, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sensor_tick_ms, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        // Kept so older boot.py calls do not fail. EvoDownloadManager no longer uses them.
        { MP_QSTR_console,        MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_max_chunk,      MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t vals[9];
    mp_arg_parse_all(n_args, args, kw_args, 9, allowed_args, vals);

    mp_obj_t cfg = load_config();
    mp_obj_t download = get_download_config(cfg);

    bool should_start = false;

    if (vals[ARG_start_download].u_obj != MP_OBJ_NULL) {
        should_start = should_start || obj_to_bool(vals[ARG_start_download].u_obj, false);
    }

    if (vals[ARG_start_ble].u_obj != MP_OBJ_NULL) {
        should_start = should_start || obj_to_bool(vals[ARG_start_ble].u_obj, false);
    }

    if (!should_start) {
        return mp_const_none;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t manager = import_module("EvoDownloadManager");
        mp_obj_t start = mp_load_attr(manager, MP_QSTR_start);

        mp_obj_t name = dict_get_default(cfg, MP_QSTR_name, new_str(EVO_DEFAULT_NAME));

        int adv_interval_us = obj_to_int(
            vals[ARG_adv_interval_us].u_obj != MP_OBJ_NULL
                ? vals[ARG_adv_interval_us].u_obj
                : dict_get_default(download, MP_QSTR_adv_interval_us, mp_obj_new_int(EVO_DOWNLOAD_ADV_INTERVAL_US)),
            EVO_DOWNLOAD_ADV_INTERVAL_US,
            20000
        );

        bool debug = obj_to_bool(
            vals[ARG_debug].u_obj != MP_OBJ_NULL
                ? vals[ARG_debug].u_obj
                : dict_get_default(download, MP_QSTR_debug, mp_obj_new_bool(EVO_DOWNLOAD_DEBUG)),
            EVO_DOWNLOAD_DEBUG
        );

        int ack_every = obj_to_int(
            vals[ARG_ack_every].u_obj != MP_OBJ_NULL
                ? vals[ARG_ack_every].u_obj
                : dict_get_default(download, MP_QSTR_ack_every, mp_obj_new_int(EVO_DOWNLOAD_ACK_EVERY)),
            EVO_DOWNLOAD_ACK_EVERY,
            1
        );

        bool sensor_streaming = obj_to_bool(
            vals[ARG_sensor_streaming].u_obj != MP_OBJ_NULL
                ? vals[ARG_sensor_streaming].u_obj
                : dict_get_default(download, MP_QSTR_sensor_streaming, mp_obj_new_bool(EVO_DOWNLOAD_SENSOR_STREAMING)),
            EVO_DOWNLOAD_SENSOR_STREAMING
        );

        int sensor_tick_ms = obj_to_int(
            vals[ARG_sensor_tick_ms].u_obj != MP_OBJ_NULL
                ? vals[ARG_sensor_tick_ms].u_obj
                : dict_get_default(download, MP_QSTR_sensor_tick_ms, mp_obj_new_int(EVO_DOWNLOAD_SENSOR_TICK_MS)),
            EVO_DOWNLOAD_SENSOR_TICK_MS,
            20
        );

        mp_obj_t call_args[12] = {
            MP_OBJ_NEW_QSTR(MP_QSTR_name), name,
            MP_OBJ_NEW_QSTR(MP_QSTR_adv_interval_us), mp_obj_new_int(adv_interval_us),
            MP_OBJ_NEW_QSTR(MP_QSTR_debug), mp_obj_new_bool(debug),
            MP_OBJ_NEW_QSTR(MP_QSTR_ack_every), mp_obj_new_int(ack_every),
            MP_OBJ_NEW_QSTR(MP_QSTR_sensor_streaming), mp_obj_new_bool(sensor_streaming),
            MP_OBJ_NEW_QSTR(MP_QSTR_sensor_tick_ms), mp_obj_new_int(sensor_tick_ms),
        };

        mp_call_function_n_kw(start, 0, 6, call_args);
        nlr_pop();
    } else {
        mp_printf(&mp_plat_print, "EvoDownloadManager start failed\n");
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(evo_init_obj, 0, evo_init);

static mp_obj_t evo_start_download(void) {
    mp_obj_t cfg = load_config();
    mp_obj_t download = get_download_config(cfg);

    bool enabled = obj_to_bool(dict_get_default(cfg, MP_QSTR_bluetooth_enabled,
        mp_obj_new_bool(EVO_BLUETOOTH_ENABLED)), EVO_BLUETOOTH_ENABLED);
    bool start_on_boot = obj_to_bool(dict_get_default(download, MP_QSTR_start_on_boot,
        mp_obj_new_bool(EVO_DOWNLOAD_START_ON_BOOT)), EVO_DOWNLOAD_START_ON_BOOT);

    if (!enabled || !start_on_boot) {
        return mp_const_false;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t manager = import_module("EvoDownloadManager");
        mp_call_function_0(mp_load_attr(manager, MP_QSTR_start));
        nlr_pop();
        return mp_const_true;
    }

    mp_printf(&mp_plat_print, "EvoDownloadManager start failed\n");
    return mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_0(evo_start_download_obj, evo_start_download);

// ============================================================================
// Unified log(*args, sep=" ", end="\n", stream="stdout")
// Always prints to serial and, if available, also forwards to EvoDownloadManager.
// ============================================================================

static mp_obj_t evo_log(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    enum {
        ARG_sep,
        ARG_end,
        ARG_stream,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sep,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_end,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_stream, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t vals[3];
    mp_arg_parse_all(0, NULL, kw_args, 3, allowed_args, vals);

    const char *sep = " ";
    const char *end = "\n";
    const char *stream = "stdout";

    if (vals[ARG_sep].u_obj != MP_OBJ_NULL) {
        sep = mp_obj_str_get_str(vals[ARG_sep].u_obj);
    }
    if (vals[ARG_end].u_obj != MP_OBJ_NULL) {
        end = mp_obj_str_get_str(vals[ARG_end].u_obj);
    }
    if (vals[ARG_stream].u_obj != MP_OBJ_NULL) {
        stream = mp_obj_str_get_str(vals[ARG_stream].u_obj);
    }

    vstr_t vstr;
    vstr_init(&vstr, 64);

    mp_print_t vstr_pr = { &vstr, (mp_print_strn_t)vstr_add_strn };

    for (size_t i = 0; i < n_args; i++) {
        if (i > 0) {
            vstr_add_str(&vstr, sep);
        }
        mp_obj_print_helper(&vstr_pr, args[i], PRINT_STR);
    }
    vstr_add_str(&vstr, end);

    const char *msg = vstr_null_terminated_str(&vstr);

    mp_printf(&mp_plat_print, "%s", msg);

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t manager = import_module("EvoDownloadManager");

        mp_obj_t dest[2];
        mp_load_method(manager, MP_QSTR_console_write, dest);

        mp_obj_t meth_args[4] = {
            dest[0],
            dest[1],
            mp_obj_new_str(msg, vstr.len),
            mp_obj_new_str(stream, strlen(stream)),
        };
        mp_call_method_n_kw(1, 1, meth_args);

        nlr_pop();
    }

    vstr_clear(&vstr);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(evo_log_obj, 0, evo_log);

// ============================================================================
// Getters
// ============================================================================

static mp_obj_t get_name(void) {
    mp_obj_t cfg = load_config();
    return dict_get_default(cfg, MP_QSTR_name, new_str(EVO_DEFAULT_NAME));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_name_obj, get_name);

static mp_obj_t get_controller_type(void) {
    mp_obj_t cfg = load_config();
    return dict_get_default(cfg, MP_QSTR_controller_type, new_str(EVO_CONTROLLER_TYPE));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_controller_type_obj, get_controller_type);

static mp_obj_t get_download_config_public(void) {
    mp_obj_t cfg = load_config();
    return get_download_config(cfg);
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_download_config_obj, get_download_config_public);

static mp_obj_t get_bluetooth_enabled(void) {
    mp_obj_t cfg = load_config();
    mp_obj_t value = dict_get_default(cfg, MP_QSTR_bluetooth_enabled,
        mp_obj_new_bool(EVO_BLUETOOTH_ENABLED));
    return mp_obj_new_bool(obj_to_bool(value, EVO_BLUETOOTH_ENABLED));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_bluetooth_enabled_obj, get_bluetooth_enabled);

static mp_obj_t get_multiple_program_filesystem(void) {
    mp_obj_t cfg = load_config();
    mp_obj_t value = dict_get_default(cfg, MP_QSTR_multiple_program_filesystem,
        mp_obj_new_bool(EVO_MULTIPLE_PROGRAM_FILESYSTEM));
    return mp_obj_new_bool(obj_to_bool(value, EVO_MULTIPLE_PROGRAM_FILESYSTEM));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_multiple_program_filesystem_obj, get_multiple_program_filesystem);

static const MP_DEFINE_STR_OBJ(evo_firmware_version_obj, MICROPY_GIT_TAG " on " MICROPY_BUILD_DATE EVO_MICROPY_BUILD_TYPE_PAREN);

static mp_obj_t get_firmware_version(void) {
    return MP_OBJ_FROM_PTR(&evo_firmware_version_obj);
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_firmware_version_obj, get_firmware_version);

static mp_obj_t get_config(void) {
    return load_config();
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_config_obj, get_config);

static mp_obj_t reset_pwm(void) {
    evo_pwm_obj_t *pwm = MP_OBJ_TO_PTR(evo_get_pwm_singleton());
    evo_pwm_reset(pwm);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(reset_pwm_obj, reset_pwm);

// ============================================================================
// Setters
// ============================================================================

static void maybe_reset(bool reset) {
    if (!reset) {
        return;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t machine = import_module("machine");
        mp_call_function_0(mp_load_attr(machine, MP_QSTR_reset));
        nlr_pop();
    }
}

static mp_obj_t set_name(size_t n_args, const mp_obj_t *args) {
    mp_obj_t name = args[0];
    bool reset = true;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    if (!is_valid_name(name)) {
        mp_printf(&mp_plat_print,
            "Invalid name. Use only A-Z, a-z, 0-9, _ or -, max %d chars.\n",
            MAX_NAME_LEN);
        return mp_const_false;
    }

    mp_obj_t cfg = load_config();
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_name), name);

    if (!safe_write_config(cfg)) {
        mp_printf(&mp_plat_print, "Config write failed\n");
        return mp_const_false;
    }

    maybe_reset(reset);
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_name_obj, 1, 2, set_name);

static mp_obj_t set_controller_type(size_t n_args, const mp_obj_t *args) {
    mp_obj_t ct = args[0];
    bool reset = false;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    if (!mp_obj_is_str(ct)) {
        mp_printf(&mp_plat_print, "Invalid controller type\n");
        return mp_const_false;
    }

    size_t len = 0;
    mp_obj_str_get_data(ct, &len);
    if (len == 0) {
        mp_printf(&mp_plat_print, "Invalid controller type\n");
        return mp_const_false;
    }

    mp_obj_t cfg = load_config();
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_controller_type), ct);

    if (!safe_write_config(cfg)) {
        mp_printf(&mp_plat_print, "Config write failed\n");
        return mp_const_false;
    }

    maybe_reset(reset);
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_controller_type_obj, 1, 2, set_controller_type);

static mp_obj_t set_multiple_program_filesystem(size_t n_args, const mp_obj_t *args) {
    bool on = mp_obj_is_true(args[0]);
    bool reset = false;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    mp_obj_t cfg = load_config();
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_multiple_program_filesystem), mp_obj_new_bool(on));

    if (!safe_write_config(cfg)) {
        mp_printf(&mp_plat_print, "Config write failed\n");
        return mp_const_false;
    }

    maybe_reset(reset);
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_multiple_program_filesystem_obj, 1, 2, set_multiple_program_filesystem);

static mp_obj_t set_download_start_on_boot(size_t n_args, const mp_obj_t *args) {
    bool on = mp_obj_is_true(args[0]);
    bool reset = false;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    if (!save_download_value(MP_QSTR_start_on_boot, mp_obj_new_bool(on))) {
        return mp_const_false;
    }

    maybe_reset(reset);
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_download_start_on_boot_obj, 1, 2, set_download_start_on_boot);

static mp_obj_t set_bluetooth_enabled(size_t n_args, const mp_obj_t *args) {
    bool on = mp_obj_is_true(args[0]);
    bool reset = false;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    mp_obj_t cfg = load_config();
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_bluetooth_enabled), mp_obj_new_bool(on));

    if (!safe_write_config(cfg)) {
        return mp_const_false;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t manager = import_module("EvoDownloadManager");

        if (on) {
            mp_call_function_0(mp_load_attr(manager, MP_QSTR_start));
        } else {
            mp_call_function_0(mp_load_attr(manager, MP_QSTR_stop));
        }

        nlr_pop();
    } else {
        mp_printf(&mp_plat_print, "Bluetooth state change failed\n");
    }

    maybe_reset(reset);
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_bluetooth_enabled_obj, 1, 2, set_bluetooth_enabled);

// ============================================================================
// Module
// ============================================================================

static const mp_rom_map_elem_t evo_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_evo) },

    { MP_ROM_QSTR(MP_QSTR_evo_init), MP_ROM_PTR(&evo_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_evo_start_download), MP_ROM_PTR(&evo_start_download_obj) },
    { MP_ROM_QSTR(MP_QSTR_log), MP_ROM_PTR(&evo_log_obj) },

    { MP_ROM_QSTR(MP_QSTR_get_name), MP_ROM_PTR(&get_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_controller_type), MP_ROM_PTR(&get_controller_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_download_config), MP_ROM_PTR(&get_download_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_bluetooth_enabled), MP_ROM_PTR(&get_bluetooth_enabled_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_multiple_program_filesystem), MP_ROM_PTR(&get_multiple_program_filesystem_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_firmware_version), MP_ROM_PTR(&get_firmware_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_config), MP_ROM_PTR(&get_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_pwm), MP_ROM_PTR(&reset_pwm_obj) },

    { MP_ROM_QSTR(MP_QSTR_set_name), MP_ROM_PTR(&set_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_controller_type), MP_ROM_PTR(&set_controller_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_multiple_program_filesystem), MP_ROM_PTR(&set_multiple_program_filesystem_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_bluetooth_enabled), MP_ROM_PTR(&set_bluetooth_enabled_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_download_start_on_boot), MP_ROM_PTR(&set_download_start_on_boot_obj) },

    { MP_ROM_QSTR(MP_QSTR_is_pressed), MP_ROM_PTR(&evo_is_pressed_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_released), MP_ROM_PTR(&evo_is_released_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_bumped), MP_ROM_PTR(&evo_is_bumped_obj) },

    { MP_ROM_QSTR(MP_QSTR_EVOPWMDriver), MP_ROM_PTR(&evo_pwm_type) },

    { MP_ROM_QSTR(MP_QSTR_EvoMotor), MP_ROM_PTR(&evo_motor_type) },
    { MP_ROM_QSTR(MP_QSTR_EvoMotorPair), MP_ROM_PTR(&evo_motorpair_type) },
    { MP_ROM_QSTR(MP_QSTR_EvoMecanum), MP_ROM_PTR(&evo_mecanum_type) },

    { MP_ROM_QSTR(MP_QSTR_M1), MP_ROM_INT(EVO_M1) },
    { MP_ROM_QSTR(MP_QSTR_M2), MP_ROM_INT(EVO_M2) },
    { MP_ROM_QSTR(MP_QSTR_M3), MP_ROM_INT(EVO_M3) },
    { MP_ROM_QSTR(MP_QSTR_M4), MP_ROM_INT(EVO_M4) },

    { MP_ROM_QSTR(MP_QSTR_GENERICWITHENCODER),    MP_ROM_INT(MT_GENERICWITHENCODER) },
    { MP_ROM_QSTR(MP_QSTR_GENERICWITHOUTENCODER), MP_ROM_INT(MT_GENERICWITHOUTENCODER) },
    { MP_ROM_QSTR(MP_QSTR_EV3LargeMotor),         MP_ROM_INT(MT_EV3LargeMotor) },
    { MP_ROM_QSTR(MP_QSTR_EV3MediumMotor),        MP_ROM_INT(MT_EV3MediumMotor) },
    { MP_ROM_QSTR(MP_QSTR_GeekServoDCMotor),      MP_ROM_INT(MT_GeekServoDCMotor) },
    { MP_ROM_QSTR(MP_QSTR_ITERMK495),             MP_ROM_INT(MT_ITERMK495) },
    { MP_ROM_QSTR(MP_QSTR_ITERMK330),             MP_ROM_INT(MT_ITERMK330) },
    { MP_ROM_QSTR(MP_QSTR_ITERMK195),             MP_ROM_INT(MT_ITERMK195) },
    { MP_ROM_QSTR(MP_QSTR_EVOMotor300),           MP_ROM_INT(MT_EVOMotor300) },
    { MP_ROM_QSTR(MP_QSTR_EVOMotor100),           MP_ROM_INT(MT_EVOMotor100) },

    { MP_ROM_QSTR(MP_QSTR_COAST), MP_ROM_INT(EVO_STOP_COAST) },
    { MP_ROM_QSTR(MP_QSTR_BRAKE), MP_ROM_INT(EVO_STOP_BRAKE) },
    { MP_ROM_QSTR(MP_QSTR_HOLD),  MP_ROM_INT(EVO_STOP_HOLD)  },

    { MP_ROM_QSTR(MP_QSTR_ACCEL_NONE),      MP_ROM_INT(EVO_ACCEL_NONE) },
    { MP_ROM_QSTR(MP_QSTR_ACCEL_TRAPEZOID), MP_ROM_INT(EVO_ACCEL_TRAPEZOID) },
    { MP_ROM_QSTR(MP_QSTR_ACCEL_SCURVE),    MP_ROM_INT(EVO_ACCEL_SCURVE) },
};

static MP_DEFINE_CONST_DICT(evo_module_globals, evo_module_globals_table);

const mp_obj_module_t mp_module_evo = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&evo_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_evo, mp_module_evo);
MP_REGISTER_MODULE(MP_QSTR__evo, mp_module_evo);
