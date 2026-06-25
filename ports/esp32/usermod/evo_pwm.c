#include "py/runtime.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "py/qstr.h"
#include "py/mpstate.h"
#include "py/nlr.h"

#include "evo_pwm.h"

#define EVO_PWM_I2C_RETRIES 3
#define EVO_PWM_I2C_RETRY_DELAY_MS 2

// Forward declaration for use before MP_DEFINE_CONST_OBJ_TYPE(...)
extern const mp_obj_type_t evo_pwm_type;
static mp_obj_t get_board_I2CB(void);

// GC-rooted singleton
MP_REGISTER_ROOT_POINTER(mp_obj_t evo_pwm_singleton);

static void safe_i2c_writeto_mem(mp_obj_t i2c, uint16_t addr, uint8_t memaddr, const uint8_t *buf, size_t len) {
    (void)i2c;

    for (int attempt = 0; attempt < EVO_PWM_I2C_RETRIES; attempt++) {
        nlr_buf_t nlr;
        if (nlr_push(&nlr) == 0) {
            mp_obj_t helper = mp_import_name(qstr_from_str("_evo_pwm"), mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
            mp_obj_t write = mp_load_attr(helper, qstr_from_str("writeto_mem"));

            mp_call_function_3(
                write,
                mp_obj_new_int(addr),
                mp_obj_new_int(memaddr),
                mp_obj_new_bytes(buf, len)
            );
            nlr_pop();
            return;
        }

        if (attempt + 1 >= EVO_PWM_I2C_RETRIES) {
            nlr_jump(nlr.ret_val);
        }

        MICROPY_EVENT_POLL_HOOK;
        mp_hal_delay_ms(EVO_PWM_I2C_RETRY_DELAY_MS);
    }
}

void evo_pwm_set_raw(evo_pwm_obj_t *pwm, uint8_t ch, int on, int off) {
    uint8_t buf[4] = {
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0xFF),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0xFF),
    };

    mp_obj_t i2c = get_board_I2CB();

    safe_i2c_writeto_mem(
        i2c,
        pwm->addr,
        (uint8_t)(PCA9685_LED0_ON_L + 4 * ch),
        buf,
        4
    );
}

static mp_obj_t import_board_pins(void) {
    return mp_import_name(MP_QSTR_pins, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
}

static mp_obj_t get_board_I2CB(void) {
    mp_obj_t pins = import_board_pins();
    return mp_load_attr(pins, qstr_from_str("I2CB"));
}

static uint16_t get_board_pwm_addr(void) {
    mp_obj_t pins = import_board_pins();
    return (uint16_t)mp_obj_get_int(mp_load_attr(pins, qstr_from_str("PCA9685PW_ADDRESS")));
}

void evo_pwm_reset(evo_pwm_obj_t *pwm) {
    mp_obj_t i2c = get_board_I2CB();

    uint8_t mode2 = PCA9685_MODE2_OUTDRV;
    safe_i2c_writeto_mem(i2c, pwm->addr, PCA9685_MODE2, &mode2, 1);

    uint8_t mode1 = PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
    safe_i2c_writeto_mem(i2c, pwm->addr, PCA9685_MODE1, &mode1, 1);
    mp_hal_delay_ms(5);

    pwm->freq_hz = 0;
}

void evo_pwm_set_freq(evo_pwm_obj_t *pwm, int hz) {
    if (hz <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("freq must be > 0"));
    }
    if (pwm->freq_hz == hz) {
        return;
    }

    int prescale = (int)(25000000.0f / 4096.0f / (float)hz + 0.5f) - 1;
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;

    mp_obj_t i2c = get_board_I2CB();

    uint8_t sleep_mode = PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL | PCA9685_MODE1_SLEEP;
    safe_i2c_writeto_mem(i2c, pwm->addr, PCA9685_MODE1, &sleep_mode, 1);

    uint8_t p = (uint8_t)prescale;
    safe_i2c_writeto_mem(i2c, pwm->addr, PCA9685_PRESCALE, &p, 1);

    uint8_t wake_mode = PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
    safe_i2c_writeto_mem(i2c, pwm->addr, PCA9685_MODE1, &wake_mode, 1);
    mp_hal_delay_ms(5);

    uint8_t restart_mode = PCA9685_MODE1_RESTART | PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
    safe_i2c_writeto_mem(i2c, pwm->addr, PCA9685_MODE1, &restart_mode, 1);
    pwm->freq_hz = hz;
}

mp_obj_t evo_get_pwm_singleton(void) {
    mp_obj_t *root = &MP_STATE_PORT(evo_pwm_singleton);
    if (*root != MP_OBJ_NULL) {
        return *root;
    }

    mp_obj_t i2cb = get_board_I2CB();

    evo_pwm_obj_t *obj = mp_obj_malloc(evo_pwm_obj_t, &evo_pwm_type);
    obj->i2c_obj = i2cb;
    obj->addr = get_board_pwm_addr();
    obj->freq_hz = 0;

    *root = MP_OBJ_FROM_PTR(obj);
    return *root;
}
MP_DEFINE_CONST_FUN_OBJ_0(evo_get_pwm_singleton_obj, evo_get_pwm_singleton);

static mp_obj_t evo_pwm_reset_method(mp_obj_t self_in) {
    evo_pwm_obj_t *self = MP_OBJ_TO_PTR(self_in);
    evo_pwm_reset(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_pwm_reset_obj, evo_pwm_reset_method);

static mp_obj_t evo_pwm_freq(size_t n_args, const mp_obj_t *args) {
    evo_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (n_args == 1) {
        return mp_obj_new_int(self->freq_hz);
    }

    evo_pwm_set_freq(self, mp_obj_get_int(args[1]));

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(evo_pwm_freq_obj, 1, 2, evo_pwm_freq);

static mp_obj_t evo_pwm_pwm(size_t n_args, const mp_obj_t *args) {
    evo_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int ch  = mp_obj_get_int(args[1]);
    int on  = mp_obj_get_int(args[2]);
    int off = mp_obj_get_int(args[3]);

    if (ch < 0 || ch > 15) {
        mp_raise_ValueError(MP_ERROR_TEXT("ch out of range"));
    }
    if (on < 0 || on > 4096 || off < 0 || off > 4096) {
        mp_raise_ValueError(MP_ERROR_TEXT("pwm out of range"));
    }

    evo_pwm_set_raw(self, (uint8_t)ch, on, off);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(evo_pwm_pwm_obj, 4, 4, evo_pwm_pwm);

static const mp_rom_map_elem_t evo_pwm_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&evo_pwm_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_freq), MP_ROM_PTR(&evo_pwm_freq_obj) },
    { MP_ROM_QSTR(MP_QSTR_pwm),  MP_ROM_PTR(&evo_pwm_pwm_obj)  },
};
static MP_DEFINE_CONST_DICT(evo_pwm_locals_dict, evo_pwm_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    evo_pwm_type,
    MP_QSTR_EVOPWMDriver,
    MP_TYPE_FLAG_NONE,
    locals_dict, &evo_pwm_locals_dict
);
