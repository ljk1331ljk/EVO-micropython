#include "py/runtime.h"
#include "py/mphal.h"
#include "py/obj.h"
#include "py/qstr.h"
#include "py/mpstate.h"

#include "evo_pwm.h"

// Forward declaration for use before MP_DEFINE_CONST_OBJ_TYPE(...)
extern const mp_obj_type_t evo_pwm_type;

// GC-rooted singleton
MP_REGISTER_ROOT_POINTER(mp_obj_t evo_pwm_singleton);

static void safe_i2c_writeto_mem(mp_obj_t i2c, uint16_t addr, uint8_t memaddr, const uint8_t *buf, size_t len) {
    mp_obj_t dest[5];
    mp_load_method(i2c, MP_QSTR_writeto_mem, dest);

    dest[2] = mp_obj_new_int(addr);
    dest[3] = mp_obj_new_int(memaddr);
    dest[4] = mp_obj_new_bytes(buf, len);

    mp_call_method_n_kw(3, 0, dest);
}

static uint8_t i2c_readfrom_mem_u8(mp_obj_t i2c, uint16_t addr, uint8_t memaddr) {
    mp_obj_t dest[5];
    mp_load_method(i2c, MP_QSTR_readfrom_mem, dest);

    dest[2] = mp_obj_new_int(addr);
    dest[3] = mp_obj_new_int(memaddr);
    dest[4] = mp_obj_new_int(1);

    mp_obj_t data = mp_call_method_n_kw(3, 0, dest);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len < 1) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("I2C read returned no data"));
    }

    return ((const uint8_t *)bufinfo.buf)[0];
}

void evo_pwm_set_raw(evo_pwm_obj_t *pwm, uint8_t ch, int on, int off) {
    uint8_t buf[4] = {
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0xFF),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0xFF),
    };

    safe_i2c_writeto_mem(
        pwm->i2c_obj,
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

void evo_pwm_set_freq(evo_pwm_obj_t *pwm, int hz) {
    if (hz <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("freq must be > 0"));
    }

    int prescale = (int)(25000000.0f / 4096.0f / (float)hz + 0.5f);
    if (prescale < 3) prescale = 3;
    if (prescale > 255) prescale = 255;

    uint8_t old_mode = i2c_readfrom_mem_u8(pwm->i2c_obj, pwm->addr, PCA9685_MODE1);

    uint8_t sleep_mode = (old_mode & 0x7F) | 0x10;
    safe_i2c_writeto_mem(pwm->i2c_obj, pwm->addr, PCA9685_MODE1, &sleep_mode, 1);

    uint8_t p = (uint8_t)prescale;
    safe_i2c_writeto_mem(pwm->i2c_obj, pwm->addr, PCA9685_PRESCALE, &p, 1);

    safe_i2c_writeto_mem(pwm->i2c_obj, pwm->addr, PCA9685_MODE1, &old_mode, 1);
    mp_hal_delay_us(5);

    uint8_t ai = old_mode | 0x20;
    safe_i2c_writeto_mem(pwm->i2c_obj, pwm->addr, PCA9685_MODE1, &ai, 1);
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

    uint8_t mode1 = PCA9685_MODE1_ALLCALL;
    safe_i2c_writeto_mem(obj->i2c_obj, obj->addr, PCA9685_MODE1, &mode1, 1);
    mp_hal_delay_ms(5);

    uint8_t mode2 = PCA9685_MODE2_OUTDRV;
    safe_i2c_writeto_mem(obj->i2c_obj, obj->addr, PCA9685_MODE2, &mode2, 1);

    mode1 = PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
    safe_i2c_writeto_mem(obj->i2c_obj, obj->addr, PCA9685_MODE1, &mode1, 1);
    *root = MP_OBJ_FROM_PTR(obj);
    return *root;
}
MP_DEFINE_CONST_FUN_OBJ_0(evo_get_pwm_singleton_obj, evo_get_pwm_singleton);

static mp_obj_t evo_pwm_freq(size_t n_args, const mp_obj_t *args) {
    evo_pwm_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (n_args == 1) {
        uint8_t prescale = i2c_readfrom_mem_u8(self->i2c_obj, self->addr, PCA9685_PRESCALE);
        float f = 25000000.0f / 4096.0f / ((float)prescale - 0.5f);
        return mp_obj_new_int((int)f);
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
