#pragma once
#include <stdint.h>
#include "py/obj.h"

// ---------- PCA9685-ish registers ----------
#define PCA9685_MODE1     (0x00)
#define PCA9685_MODE2     (0x01)
#define PCA9685_PRESCALE  (0xFE)
#define PCA9685_LED0_ON_L (0x06)

#define PCA9685_MODE1_RESTART (0x80)
#define PCA9685_MODE1_AI      (0x20)
#define PCA9685_MODE1_SLEEP   (0x10)
#define PCA9685_MODE1_ALLCALL (0x01)
#define PCA9685_MODE2_OUTDRV  (0x04)

#ifndef PCA9685PW_ADDRESS
#define PCA9685PW_ADDRESS (0x40)
#endif

typedef struct _evo_pwm_obj_t {
    mp_obj_base_t base;
    mp_obj_t i2c_obj;
    uint16_t addr;
    int freq_hz;
} evo_pwm_obj_t;

extern const mp_obj_type_t evo_pwm_type;
extern const mp_obj_fun_builtin_fixed_t evo_get_pwm_singleton_obj;

mp_obj_t evo_get_pwm_singleton(void);
void evo_pwm_reset(evo_pwm_obj_t *pwm);
void evo_pwm_set_freq(evo_pwm_obj_t *pwm, int hz);
void evo_pwm_set_raw(evo_pwm_obj_t *pwm, uint8_t ch, int on, int off);
