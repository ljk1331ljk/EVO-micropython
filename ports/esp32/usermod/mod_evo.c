#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/nlr.h"
#include "py/mphal.h"

#include <string.h>
#include <stdbool.h>

#include "evo_pwm.h"
#include "evo_motor.h"
#include "evo_motorpair.h"
#include "evo_mecanum.h"

#define CONFIG_FILE "/evo_config.json"
#define CONFIG_TMP  "/evo_config.json.tmp"

#define DEFAULT_NAME "Evo_X1E"
#define DEFAULT_CONTROLLER "EVO-X1E"
#define MAX_NAME_LEN 31

// ----------------------------------------------------------
// Helpers
// ----------------------------------------------------------

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
    } else {
        return default_val;
    }
}

// ----------------------------------------------------------
// Config helpers
// ----------------------------------------------------------

static mp_obj_t default_config(void) {
    mp_obj_t d = mp_obj_new_dict(4);
    mp_obj_t n = new_str(DEFAULT_NAME);

    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_name), n);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_ble_name), n);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_wifi_name), n);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_controller_type), new_str(DEFAULT_CONTROLLER));

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

        mp_call_function_2(mp_load_attr(os, MP_QSTR_rename),
            new_str(CONFIG_TMP), new_str(CONFIG_FILE));

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

        mp_obj_t raw_name = dict_get_default(raw_cfg, MP_QSTR_name, new_str(DEFAULT_NAME));
        mp_obj_t name = sanitize_name(raw_name, DEFAULT_NAME);

        mp_obj_t raw_ble = dict_get_default(raw_cfg, MP_QSTR_ble_name, name);
        mp_obj_t ble_name = sanitize_name(raw_ble, mp_obj_str_get_str(name));

        mp_obj_t raw_wifi = dict_get_default(raw_cfg, MP_QSTR_wifi_name, name);
        mp_obj_t wifi_name = sanitize_name(raw_wifi, mp_obj_str_get_str(name));

        mp_obj_t raw_ct = dict_get_default(raw_cfg, MP_QSTR_controller_type, new_str(DEFAULT_CONTROLLER));
        mp_obj_t controller_type;
        if (mp_obj_is_str(raw_ct)) {
            size_t ct_len = 0;
            mp_obj_str_get_data(raw_ct, &ct_len);
            if (ct_len == 0) {
                controller_type = new_str(DEFAULT_CONTROLLER);
            } else {
                controller_type = raw_ct;
            }
        } else {
            controller_type = new_str(DEFAULT_CONTROLLER);
        }

        mp_obj_t cfg = mp_obj_new_dict(4);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_name), name);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_ble_name), ble_name);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_wifi_name), wifi_name);
        mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_controller_type), controller_type);

        safe_write_config(cfg);

        nlr_pop();
        return cfg;
    }

    {
        mp_obj_t d = default_config();
        safe_write_config(d);
        return d;
    }
}

// ----------------------------------------------------------
// evo_init(start_ble=True, console=True, max_chunk=160, debug=False)
// ----------------------------------------------------------

static mp_obj_t evo_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    enum {
        ARG_start_ble,
        ARG_console,
        ARG_max_chunk,
        ARG_debug,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_start_ble, MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_console,   MP_ARG_BOOL, {.u_bool = true} },
        { MP_QSTR_max_chunk, MP_ARG_INT,  {.u_int = 160} },
        { MP_QSTR_debug,     MP_ARG_BOOL, {.u_bool = false} },
    };

    mp_arg_val_t vals[4];
    mp_arg_parse_all(n_args, args, kw_args, 4, allowed_args, vals);

    mp_obj_t cfg = load_config();

    if (vals[ARG_start_ble].u_bool) {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t ble = import_module("ble_uploader");
            mp_obj_t start = mp_load_attr(ble, MP_QSTR_start);

            mp_obj_t ble_name = dict_get_default(cfg, MP_QSTR_ble_name, new_str(DEFAULT_NAME));

            mp_obj_t call_args[8] = {
                MP_OBJ_NEW_QSTR(MP_QSTR_name), ble_name,
                MP_OBJ_NEW_QSTR(MP_QSTR_max_chunk), mp_obj_new_int(vals[ARG_max_chunk].u_int),
                MP_OBJ_NEW_QSTR(MP_QSTR_debug), mp_obj_new_bool(vals[ARG_debug].u_bool),
                MP_OBJ_NEW_QSTR(MP_QSTR_console), mp_obj_new_bool(vals[ARG_console].u_bool),
            };

            mp_call_function_n_kw(start, 0, 4, call_args);
            nlr_pop();
        } else {
            mp_printf(&mp_plat_print, "BLE start failed\n");
        }
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(evo_init_obj, 0, evo_init);

// ----------------------------------------------------------
// Unified log(*args, sep=" ", end="\n", stream="stdout")
// Always prints to serial and, if available, also calls
// ble_uploader.console_write(msg, stream=...)
// ----------------------------------------------------------

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

    // Always print to serial / REPL
    mp_printf(&mp_plat_print, "%s", msg);

    // Also forward to ble_uploader if available
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t ble = import_module("ble_uploader");

        mp_obj_t dest[2];
        mp_load_method(ble, qstr_from_str("console_write"), dest);

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

// ----------------------------------------------------------
// Getters
// ----------------------------------------------------------

static mp_obj_t get_name(void) {
    mp_obj_t cfg = load_config();
    return dict_get_default(cfg, MP_QSTR_name, new_str(DEFAULT_NAME));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_name_obj, get_name);

static mp_obj_t get_ble_name(void) {
    mp_obj_t cfg = load_config();
    return dict_get_default(cfg, MP_QSTR_ble_name, new_str(DEFAULT_NAME));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_ble_name_obj, get_ble_name);

static mp_obj_t get_wifi_name(void) {
    mp_obj_t cfg = load_config();
    return dict_get_default(cfg, MP_QSTR_wifi_name, new_str(DEFAULT_NAME));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_wifi_name_obj, get_wifi_name);

static mp_obj_t get_controller_type(void) {
    mp_obj_t cfg = load_config();
    return dict_get_default(cfg, MP_QSTR_controller_type, new_str(DEFAULT_CONTROLLER));
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_controller_type_obj, get_controller_type);

static mp_obj_t get_config(void) {
    return load_config();
}
static MP_DEFINE_CONST_FUN_OBJ_0(get_config_obj, get_config);

// ----------------------------------------------------------
// Setters
// ----------------------------------------------------------

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
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_ble_name), name);
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_wifi_name), name);

    if (!safe_write_config(cfg)) {
        mp_printf(&mp_plat_print, "Config write failed\n");
        return mp_const_false;
    }

    if (reset) {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t machine = import_module("machine");
            mp_call_function_0(mp_load_attr(machine, MP_QSTR_reset));
            nlr_pop();
        }
    }

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_name_obj, 1, 2, set_name);

static mp_obj_t set_ble_name(size_t n_args, const mp_obj_t *args) {
    mp_obj_t name = args[0];
    bool reset = true;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    if (!is_valid_name(name)) {
        mp_printf(&mp_plat_print,
            "Invalid BLE name. Use only A-Z, a-z, 0-9, _ or -, max %d chars.\n",
            MAX_NAME_LEN);
        return mp_const_false;
    }

    mp_obj_t cfg = load_config();
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_ble_name), name);

    if (!safe_write_config(cfg)) {
        mp_printf(&mp_plat_print, "Config write failed\n");
        return mp_const_false;
    }

    if (reset) {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t machine = import_module("machine");
            mp_call_function_0(mp_load_attr(machine, MP_QSTR_reset));
            nlr_pop();
        }
    }

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_ble_name_obj, 1, 2, set_ble_name);

static mp_obj_t set_wifi_name(size_t n_args, const mp_obj_t *args) {
    mp_obj_t name = args[0];
    bool reset = true;
    if (n_args > 1) {
        reset = mp_obj_is_true(args[1]);
    }

    if (!is_valid_name(name)) {
        mp_printf(&mp_plat_print,
            "Invalid WiFi name. Use only A-Z, a-z, 0-9, _ or -, max %d chars.\n",
            MAX_NAME_LEN);
        return mp_const_false;
    }

    mp_obj_t cfg = load_config();
    mp_obj_dict_store(cfg, MP_OBJ_NEW_QSTR(MP_QSTR_wifi_name), name);

    if (!safe_write_config(cfg)) {
        mp_printf(&mp_plat_print, "Config write failed\n");
        return mp_const_false;
    }

    if (reset) {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t machine = import_module("machine");
            mp_call_function_0(mp_load_attr(machine, MP_QSTR_reset));
            nlr_pop();
        }
    }

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_wifi_name_obj, 1, 2, set_wifi_name);

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

    if (reset) {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t machine = import_module("machine");
            mp_call_function_0(mp_load_attr(machine, MP_QSTR_reset));
            nlr_pop();
        }
    }

    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_controller_type_obj, 1, 2, set_controller_type);

// ----------------------------------------------------------
// Module
// ----------------------------------------------------------

static const mp_rom_map_elem_t evo_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_evo) },

    { MP_ROM_QSTR(MP_QSTR_evo_init), MP_ROM_PTR(&evo_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_log), MP_ROM_PTR(&evo_log_obj) },

    { MP_ROM_QSTR(MP_QSTR_get_name), MP_ROM_PTR(&get_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ble_name), MP_ROM_PTR(&get_ble_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_wifi_name), MP_ROM_PTR(&get_wifi_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_controller_type), MP_ROM_PTR(&get_controller_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_config), MP_ROM_PTR(&get_config_obj) },

    { MP_ROM_QSTR(MP_QSTR_set_name), MP_ROM_PTR(&set_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ble_name), MP_ROM_PTR(&set_ble_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_wifi_name), MP_ROM_PTR(&set_wifi_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_controller_type), MP_ROM_PTR(&set_controller_type_obj) },

    { MP_ROM_QSTR(MP_QSTR_EVOPWMDriver), MP_ROM_PTR(&evo_get_pwm_singleton_obj) },

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
