#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mpstate.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "esp_err.h"

#include "evo_motor.h"

#define EVO_MOTOR_PWM_FREQ_HZ 2500

MP_REGISTER_ROOT_POINTER(evo_motor_obj_t *evo_motor_obj_head);

static const int8_t quad_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static void IRAM_ATTR evo_quad_isr(void *arg) {
    evo_motor_obj_t *m = (evo_motor_obj_t*)arg;
    if (!m || !m->active) {
        return;
    }

    uint8_t a = (uint8_t)gpio_get_level(m->t1);
    uint8_t b = (uint8_t)gpio_get_level(m->t2);
    uint8_t new_state = (a << 1) | b;

    uint8_t idx = (m->last_state << 2) | new_state;
    int8_t delta = quad_table[idx];
    if (delta) {
        m->position += (int32_t)delta * (int32_t)m->enc_dir;
    }
    m->last_state = new_state;
}

static void ensure_gpio_isr(void) {
    static bool installed = false;
    static portMUX_TYPE isr_install_mux = portMUX_INITIALIZER_UNLOCKED;
    bool do_install = false;

    portENTER_CRITICAL(&isr_install_mux);
    if (!installed) {
        installed = true;
        do_install = true;
    }
    portEXIT_CRITICAL(&isr_install_mux);

    if (do_install) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            portENTER_CRITICAL(&isr_install_mux);
            installed = false;
            portEXIT_CRITICAL(&isr_install_mux);
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("gpio_install_isr_service failed"));
        }
    }
}

static void setup_encoder_isr(evo_motor_obj_t *m) {
    gpio_config_t io = {0};
    io.intr_type = GPIO_INTR_ANYEDGE;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = 1;
    io.pull_down_en = 0;
    io.pin_bit_mask = (1ULL << m->t1) | (1ULL << m->t2);

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("gpio_config failed"));
    }

    ensure_gpio_isr();

    uint8_t a = (uint8_t)gpio_get_level(m->t1);
    uint8_t b = (uint8_t)gpio_get_level(m->t2);
    m->last_state = (a << 1) | b;

    err = gpio_isr_handler_add(m->t1, evo_quad_isr, (void*)m);
    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("gpio_isr_handler_add t1 failed"));
    }

    err = gpio_isr_handler_add(m->t2, evo_quad_isr, (void*)m);
    if (err != ESP_OK) {
        gpio_isr_handler_remove(m->t1);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("gpio_isr_handler_add t2 failed"));
    }

    m->isr_attached = true;
    m->active = true;
}

static void detach_encoder_isr(evo_motor_obj_t *m) {
    if (!m || !m->isr_attached) {
        return;
    }

    m->active = false;
    gpio_intr_disable(m->t1);
    gpio_intr_disable(m->t2);
    gpio_isr_handler_remove(m->t1);
    gpio_isr_handler_remove(m->t2);
    m->isr_attached = false;
}

static void evo_motor_register(evo_motor_obj_t *m) {
    m->next = MP_STATE_PORT(evo_motor_obj_head);
    MP_STATE_PORT(evo_motor_obj_head) = m;
}

static void evo_motor_unregister(evo_motor_obj_t *m) {
    evo_motor_obj_t **cur = &MP_STATE_PORT(evo_motor_obj_head);
    while (*cur != NULL) {
        if (*cur == m) {
            *cur = m->next;
            m->next = NULL;
            return;
        }
        cur = &(*cur)->next;
    }
}

static void evo_motor_check(evo_motor_obj_t *m) {
    if (m == NULL || !MP_OBJ_IS_TYPE(MP_OBJ_FROM_PTR(m), &evo_motor_type) || !m->valid) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("EvoMotor is not initialised"));
    }
}

static evo_pwm_obj_t *evo_motor_get_pwm(evo_motor_obj_t *m) {
    evo_motor_check(m);
    if (m->pwm == NULL || !MP_OBJ_IS_TYPE(MP_OBJ_FROM_PTR(m->pwm), &evo_pwm_type) || !m->pwm->valid) {
        m->pwm = MP_OBJ_TO_PTR(evo_get_pwm_singleton());
    }
    return m->pwm;
}

static mp_obj_t import_board_pins(void) {
    return mp_import_name(MP_QSTR_pins, mp_const_none, MP_OBJ_NEW_SMALL_INT(0));
}


// Optional shared motor-driver nSLEEP control.
// Boards that have a common nSLEEP line should define NSLEEP_PIN in pins.py.
// Boards without NSLEEP_PIN will silently ignore enable/disable requests.
static bool s_nsleep_initialized = false;
static bool s_motor_driver_awake = false;
static gpio_num_t s_nsleep_pin = GPIO_NUM_NC;

void evo_motor_deinit_all(void) {
    evo_motor_obj_t *m = MP_STATE_PORT(evo_motor_obj_head);
    while (m != NULL) {
        if (!MP_OBJ_IS_TYPE(MP_OBJ_FROM_PTR(m), &evo_motor_type)) {
            break;
        }
        evo_motor_obj_t *next = m->next;
        detach_encoder_isr(m);
        m->pwm = NULL;
        m->valid = false;
        m->next = NULL;
        m = next;
    }
    MP_STATE_PORT(evo_motor_obj_head) = NULL;

    s_motor_driver_awake = false;
    s_nsleep_initialized = false;
    s_nsleep_pin = GPIO_NUM_NC;
}

static void evo_motor_init_nsleep(void) {
    if (s_nsleep_initialized) {
        return;
    }

    s_nsleep_initialized = true;

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t pins = import_board_pins();
        mp_obj_t nsleep_obj = mp_load_attr(pins, qstr_from_str("NSLEEP_PIN"));
        int pin = mp_obj_get_int(nsleep_obj);

        if (pin >= 0) {
            s_nsleep_pin = (gpio_num_t)pin;

            gpio_config_t io = {0};
            io.intr_type = GPIO_INTR_DISABLE;
            io.mode = GPIO_MODE_OUTPUT;
            io.pull_up_en = GPIO_PULLUP_DISABLE;
            io.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io.pin_bit_mask = (1ULL << s_nsleep_pin);

            esp_err_t err = gpio_config(&io);
            if (err != ESP_OK) {
                nlr_pop();
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("NSLEEP_PIN gpio_config failed"));
            }

            // Keep motor drivers asleep until a motor command needs them.
            gpio_set_level(s_nsleep_pin, 0);
            s_motor_driver_awake = false;
        }

        nlr_pop();
    } else {
        // NSLEEP_PIN is optional. If it is not defined, this board simply
        // has no shared motor-driver sleep control.
        s_nsleep_pin = GPIO_NUM_NC;
        s_motor_driver_awake = false;
    }
}

void evo_motor_enable_all(void) {
    evo_motor_init_nsleep();

    if (s_nsleep_pin != GPIO_NUM_NC) {
        gpio_set_level(s_nsleep_pin, 1);
        s_motor_driver_awake = true;
    }
}

void evo_motor_disable_all(void) {
    evo_motor_init_nsleep();

    if (s_nsleep_pin != GPIO_NUM_NC) {
        gpio_set_level(s_nsleep_pin, 0);
        s_motor_driver_awake = false;
    }
}

static inline void evo_motor_wake_drivers(void) {
    if (!s_motor_driver_awake) {
        evo_motor_enable_all();
    }
}

static mp_obj_t dict_get_required(mp_obj_t dict, mp_obj_t key, const char *msg) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t value = mp_obj_dict_get(dict, key);
        nlr_pop();
        return value;
    }
    mp_raise_ValueError(MP_ERROR_TEXT(msg));
}

static int dict_get_required_int(mp_obj_t dict, const char *key, const char *msg) {
    return mp_obj_get_int(dict_get_required(dict, mp_obj_new_str(key, strlen(key)), msg));
}

static void resolve_port(int port, bool motorFlip, uint8_t *p1, uint8_t *p2, gpio_num_t *t1, gpio_num_t *t2) {
    mp_obj_t pins = import_board_pins();
    mp_obj_t motor_ports = mp_load_attr(pins, qstr_from_str("MOTOR_PORTS"));
    mp_obj_t port_cfg = dict_get_required(
        motor_ports,
        mp_obj_new_int(port),
        "motor port not available on this board"
    );

    int pwm1 = dict_get_required_int(port_cfg, "pwm1", "MOTOR_PORTS entry missing pwm1");
    int pwm2 = dict_get_required_int(port_cfg, "pwm2", "MOTOR_PORTS entry missing pwm2");
    int tach1 = dict_get_required_int(port_cfg, "tach1", "MOTOR_PORTS entry missing tach1");
    int tach2 = dict_get_required_int(port_cfg, "tach2", "MOTOR_PORTS entry missing tach2");

    if (pwm1 < 0 || pwm1 > 15 || pwm2 < 0 || pwm2 > 15) {
        mp_raise_ValueError(MP_ERROR_TEXT("motor PWM channel out of range"));
    }

    if (motorFlip) {
        *p1 = (uint8_t)pwm2;
        *p2 = (uint8_t)pwm1;
        *t1 = (gpio_num_t)tach2;
        *t2 = (gpio_num_t)tach1;
    } else {
        *p1 = (uint8_t)pwm1;
        *p2 = (uint8_t)pwm2;
        *t1 = (gpio_num_t)tach1;
        *t2 = (gpio_num_t)tach2;
    }
}

static void preset(int motortype, bool flip_in, bool *flip_eff, int *cpr, int8_t *enc_dir) {
    switch (motortype) {
        case MT_GENERICWITHENCODER:
            *flip_eff = !flip_in;
            *cpr = 360;
            *enc_dir = 1;
            break;

        case MT_GENERICWITHOUTENCODER:
            // No encoder is present. Keep cpr/enc_dir at 0 so encoder-derived
            // angle/speed return 0 and setup skips attaching GPIO interrupts.
            *flip_eff = !flip_in;
            *cpr = 0;
            *enc_dir = 0;
            break;

        case MT_EV3LargeMotor:
            *flip_eff = !flip_in;
            *cpr = 720;
            *enc_dir = -1;
            break;

        case MT_EV3MediumMotor:
            *flip_eff = !flip_in;
            *cpr = 720;
            *enc_dir = -1;   // flipped by default
            break;

        case MT_GeekServoDCMotor:
            *flip_eff = flip_in;
            *cpr = 360;
            *enc_dir = 1;
            break;

        case MT_ITERMK495:
            *flip_eff = !flip_in;
            *cpr = 1204;
            *enc_dir = 1;
            break;

        case MT_ITERMK330:
            *flip_eff = !flip_in;
            *cpr = 1807;
            *enc_dir = 1;
            break;

        case MT_ITERMK195:
            *flip_eff = !flip_in;
            *cpr = 2988;
            *enc_dir = 1;
            break;

        case MT_EVOMotor300:
            *flip_eff = !flip_in;
            *cpr = 2800;
            *enc_dir = -1;
            break;

        case MT_EVOMotor100:
            *flip_eff = !flip_in;
            *cpr = 8400;
            *enc_dir = -1;
            break;

        default:
            *flip_eff = !flip_in;
            *cpr = 360;
            *enc_dir = 1;
            break;
    }
}

static inline int clamp_power(int s) {
    if (s > EVO_PWM_MAX) return EVO_PWM_MAX;
    if (s < -EVO_PWM_MAX) return -EVO_PWM_MAX;
    return s;
}

static inline void motor_prepare_pwm(evo_motor_obj_t *m) {
    evo_pwm_set_freq(evo_motor_get_pwm(m), EVO_MOTOR_PWM_FREQ_HZ);
}

// PCA9685 special full-on/full-off bit. Keep normal motor power scaled to
// 0..4095, but use 4096 when a channel must be explicitly driven fully on/off.
#define EVO_PWM_FULL 4096

static inline void motor_pwm_off(evo_motor_obj_t *m, uint8_t ch) {
    evo_pwm_set_raw(evo_motor_get_pwm(m), ch, 0, EVO_PWM_FULL);
}

static inline void motor_pwm_full_on(evo_motor_obj_t *m, uint8_t ch) {
    evo_pwm_set_raw(evo_motor_get_pwm(m), ch, EVO_PWM_FULL, 0);
}

static inline void motor_coast(evo_motor_obj_t *m) {
    motor_prepare_pwm(m);
    motor_pwm_off(m, m->power1);
    motor_pwm_off(m, m->power2);
}

static inline void motor_brake(evo_motor_obj_t *m) {
    evo_motor_wake_drivers();
    motor_prepare_pwm(m);
    motor_pwm_full_on(m, m->power1);
    motor_pwm_full_on(m, m->power2);
}

static inline void motor_hold(evo_motor_obj_t *m) {
    evo_motor_wake_drivers();
    motor_brake(m);
}

static inline void motor_stop_by_behaviour(evo_motor_obj_t *m) {
    if (m->stop_behaviour == EVO_STOP_HOLD) {
        motor_hold(m);
    } else if (m->stop_behaviour == EVO_STOP_BRAKE) {
        motor_brake(m);
    } else {
        motor_coast(m);
    }
}

static inline void evo_motor_cancel_hold(evo_motor_obj_t *m) {
    // HOLD is currently implemented as active braking only, not a background
    // position-hold controller. This helper exists so movement functions can
    // safely cancel a future hold controller without creating a missing symbol.
    (void)m;
}

void evo_motor_run_power_c(evo_motor_obj_t *m, int power) {
    evo_motor_check(m);

    int s = clamp_power(power);

    if (s != 0) {
        evo_motor_wake_drivers();
    }

    motor_prepare_pwm(m);

    if (s > 0) {
        evo_pwm_set_raw(evo_motor_get_pwm(m), m->power1, 0, s);
        motor_pwm_off(m, m->power2);
    } else if (s < 0) {
        motor_pwm_off(m, m->power1);
        evo_pwm_set_raw(evo_motor_get_pwm(m), m->power2, 0, -s);
    } else {
        motor_stop_by_behaviour(m);
    }
}

void evo_motor_brake_c(evo_motor_obj_t *m) {
    motor_brake(m);
}

void evo_motor_coast_c(evo_motor_obj_t *m) {
    motor_coast(m);
}

void evo_motor_hold_c(evo_motor_obj_t *m) {
    motor_hold(m);
}

void evo_motor_set_stop_behaviour_c(evo_motor_obj_t *m, uint8_t beh) {
    evo_motor_check(m);

    if (beh == EVO_STOP_COAST || beh == EVO_STOP_BRAKE || beh == EVO_STOP_HOLD) {
        m->stop_behaviour = beh;
    }
}

void evo_motor_reset_angle(evo_motor_obj_t *m) {
    evo_motor_check(m);
    m->position = 0;
}

int32_t evo_motor_get_angle_deg(evo_motor_obj_t *m) {
    evo_motor_check(m);

    if (m->cpr == 0) {
        return 0;
    }

    int32_t pos = m->position;
    int32_t cpr = m->cpr;

    if (pos >= 0) {
        return (pos * 360 + cpr / 2) / cpr;
    } else {
        return (pos * 360 - cpr / 2) / cpr;
    }
}



static inline int sign_from_float(mp_float_t v) {
    if (v > 0) return 1;
    if (v < 0) return -1;
    return 0;
}

static void evo_motor_reset_speed_state(evo_motor_obj_t *m) {
    m->speed_last_position = m->position;
    m->speed_last_ms = mp_hal_ticks_ms();
    m->speed_dt = 0;
    m->speed_cps = 0;
    m->speed_dps = 0;
    m->speed_integral = 0;
    m->speed_last_error = 0;
    m->speed_power = 0;
}

void evo_motor_update_speed_c(evo_motor_obj_t *m) {
    evo_motor_check(m);

    uint32_t now = mp_hal_ticks_ms();

    if (m->speed_last_ms == 0) {
        evo_motor_reset_speed_state(m);
        return;
    }

    uint32_t dt_ms = (uint32_t)(now - m->speed_last_ms);
    if (dt_ms == 0 || dt_ms < m->speed_min_update_ms) {
        return;
    }

    int32_t now_pos = m->position;
    int32_t delta = now_pos - m->speed_last_position;
    mp_float_t dt = (mp_float_t)dt_ms / 1000.0f;

    m->speed_dt = dt;
    m->speed_cps = (mp_float_t)delta / dt;

    if (m->cpr > 0) {
        m->speed_dps = m->speed_cps * 360.0f / (mp_float_t)m->cpr;
    } else {
        m->speed_dps = 0;
    }

    m->speed_last_position = now_pos;
    m->speed_last_ms = now;
}

mp_float_t evo_motor_get_speed_cps_c(evo_motor_obj_t *m) {
    evo_motor_check(m);
    return m->speed_cps;
}

mp_float_t evo_motor_get_speed_dps_c(evo_motor_obj_t *m) {
    evo_motor_check(m);
    return m->speed_dps;
}

void evo_motor_set_speed_pid_c(evo_motor_obj_t *m, mp_float_t kp, mp_float_t ki, mp_float_t kd) {
    evo_motor_check(m);

    m->speed_kp = kp;
    m->speed_ki = ki;
    m->speed_kd = kd;
    m->speed_integral = 0;
    m->speed_last_error = 0;
}

void evo_motor_set_speed_limits_c(evo_motor_obj_t *m, int min_power, int max_power) {
    evo_motor_check(m);

    if (min_power < 0 || max_power <= 0 || min_power > max_power || max_power > EVO_PWM_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid speed limits"));
    }

    m->speed_min_power = min_power;
    m->speed_max_power = max_power;
}

void evo_motor_run_speed_control_c(evo_motor_obj_t *m, mp_float_t target_dps) {
    evo_motor_check(m);

    if (target_dps == 0) {
        m->speed_integral = 0;
        m->speed_last_error = 0;
        m->speed_power = 0;
        evo_motor_update_speed_c(m);
        motor_stop_by_behaviour(m);
        return;
    }

    evo_motor_update_speed_c(m);

    mp_float_t dt = m->speed_dt;
    if (dt <= 0) {
        dt = 0.01f;
    }

    mp_float_t error = target_dps - m->speed_dps;

    m->speed_integral += error * dt;

    // Anti-windup. The value is intentionally conservative because power is limited to EVO_PWM_MAX.
    if (m->speed_integral > m->speed_integral_limit) {
        m->speed_integral = m->speed_integral_limit;
    } else if (m->speed_integral < -m->speed_integral_limit) {
        m->speed_integral = -m->speed_integral_limit;
    }

    mp_float_t derivative = (error - m->speed_last_error) / dt;
    m->speed_last_error = error;

    int dir = sign_from_float(target_dps);

    // Feed-forward start power overcomes static friction.
    // PID then adds or subtracts power to reach the requested motor speed in degrees per second.
    mp_float_t pid =
        (m->speed_kp * error) +
        (m->speed_ki * m->speed_integral) +
        (m->speed_kd * derivative);

    int power = (int)((mp_float_t)(dir * m->speed_min_power) + pid);

    // Keep the command predictable for simple motor-pair movement.
    if (dir > 0 && power < 0 && m->speed_cps >= 0) {
        power = 0;
    } else if (dir < 0 && power > 0 && m->speed_cps <= 0) {
        power = 0;
    }

    if (power > m->speed_max_power) {
        power = m->speed_max_power;
    } else if (power < -m->speed_max_power) {
        power = -m->speed_max_power;
    }

    if (power != 0 && fabsf((float)power) < (mp_float_t)m->speed_min_power) {
        power = dir * m->speed_min_power;
    }

    m->speed_power = power;
    evo_motor_run_power_c(m, power);
}

static mp_obj_t evo_motor_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_port, ARG_type, ARG_flip };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_port, MP_ARG_INT,  {.u_int = EVO_M1} },
        { MP_QSTR_type, MP_ARG_INT,  {.u_int = MT_GENERICWITHENCODER} },
        { MP_QSTR_flip, MP_ARG_BOOL, {.u_bool = false} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed), allowed, args);

    mp_obj_t pwm_obj = evo_get_pwm_singleton();
    evo_pwm_obj_t *pwm = MP_OBJ_TO_PTR(pwm_obj);

    evo_motor_obj_t *m = mp_obj_malloc(evo_motor_obj_t, type);
    m->pwm = pwm;
    m->valid = true;
    m->next = NULL;
    m->position = 0;
    m->enc_dir = 1;
    m->isr_attached = false;
    m->active = false;
    m->stop_behaviour = EVO_STOP_COAST;

    m->speed_last_position = 0;
    m->speed_last_ms = 0;
    m->speed_min_update_ms = 2;
    m->speed_dt = 0;
    m->speed_cps = 0;
    m->speed_dps = 0;

    // Conservative defaults. Tune from MicroPython with setSpeedPID() and setSpeedLimits().
    m->speed_kp = 0.6f;
    m->speed_ki = 0.02f;
    m->speed_kd = 0.0f;
    m->speed_integral = 0;
    m->speed_integral_limit = 3000.0f;
    m->speed_last_error = 0;
    m->speed_power = 0;
    m->speed_min_power = 120;
    m->speed_max_power = EVO_PWM_MAX;

    bool flip_eff;
    int cpr;
    int8_t enc_dir;
    preset(args[ARG_type].u_int, args[ARG_flip].u_bool, &flip_eff, &cpr, &enc_dir);

    m->cpr = cpr;
    m->enc_dir = enc_dir;


    resolve_port(args[ARG_port].u_int, flip_eff, &m->power1, &m->power2, &m->t1, &m->t2);
    if (m->cpr > 0 && m->enc_dir != 0) {
        setup_encoder_isr(m);
        evo_motor_register(m);
    }
    evo_motor_reset_speed_state(m);

    return MP_OBJ_FROM_PTR(m);
}

static mp_obj_t evo_motor_runPower(mp_obj_t self_in, mp_obj_t power_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_run_power_c(m, mp_obj_get_int(power_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(evo_motor_runPower_obj, evo_motor_runPower);

// Backward-compatible alias. run() is raw PWM power, not speed.
static mp_obj_t evo_motor_run(mp_obj_t self_in, mp_obj_t power_in) {
    return evo_motor_runPower(self_in, power_in);
}
static MP_DEFINE_CONST_FUN_OBJ_2(evo_motor_run_obj, evo_motor_run);

static mp_obj_t evo_motor_brake(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_brake_c(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_brake_obj, evo_motor_brake);

static mp_obj_t evo_motor_coast(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_coast_c(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_coast_obj, evo_motor_coast);

static mp_obj_t evo_motor_hold(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_hold_c(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_hold_obj, evo_motor_hold);

static mp_obj_t evo_motor_setStopBehaviour(mp_obj_t self_in, mp_obj_t beh_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    int beh = mp_obj_get_int(beh_in);
    if (beh != EVO_STOP_COAST && beh != EVO_STOP_BRAKE && beh != EVO_STOP_HOLD) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid stop behaviour"));
    }
    m->stop_behaviour = (uint8_t)beh;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(evo_motor_setStopBehaviour_obj, evo_motor_setStopBehaviour);

static mp_obj_t evo_motor_setStopBehavior(mp_obj_t self_in, mp_obj_t beh_in) {
    return evo_motor_setStopBehaviour(self_in, beh_in);
}
static MP_DEFINE_CONST_FUN_OBJ_2(evo_motor_setStopBehavior_obj, evo_motor_setStopBehavior);

static mp_obj_t evo_motor_runTime(mp_obj_t self_in, mp_obj_t dps_in, mp_obj_t seconds_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);

    mp_float_t target_dps = mp_obj_get_float(dps_in);
    mp_float_t secs = mp_obj_get_float(seconds_in);
    if (secs < 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("seconds must be >= 0"));
    }

    if (target_dps == 0 || secs == 0) {
        evo_motor_run_speed_control_c(m, 0);
        return mp_const_none;
    }

    evo_motor_cancel_hold(m);
    evo_motor_reset_speed_state(m);

    uint32_t dur_ms = (uint32_t)(secs * 1000.0f + 0.5f);
    uint32_t start = mp_hal_ticks_ms();
    while ((uint32_t)(mp_hal_ticks_ms() - start) < dur_ms) {
        evo_motor_run_speed_control_c(m, target_dps);
        MICROPY_EVENT_POLL_HOOK;
        mp_hal_delay_ms(10);
    }

    evo_motor_run_speed_control_c(m, 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(evo_motor_runTime_obj, evo_motor_runTime);


static mp_obj_t evo_motor_runAngle(mp_obj_t self_in, mp_obj_t dps_in, mp_obj_t angle_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);

    mp_float_t dps = mp_obj_get_float(dps_in);
    if (dps < 0) {
        dps = -dps;
    }

    int angle = mp_obj_get_int(angle_in);
    if (angle == 0 || dps == 0) {
        evo_motor_run_speed_control_c(m, 0);
        return mp_const_none;
    }

    evo_motor_cancel_hold(m);
    evo_motor_reset_speed_state(m);

    int dir = (angle > 0) ? 1 : -1;
    int abs_angle = (angle > 0) ? angle : -angle;

    int32_t ticks = (int32_t)(((int64_t)abs_angle * (int64_t)m->cpr + 180) / 360);
    if (ticks <= 0) {
        evo_motor_run_speed_control_c(m, 0);
        return mp_const_none;
    }

    int32_t start_pos = m->position;
    int32_t target = start_pos + (int32_t)dir * ticks;
    mp_float_t target_dps = (mp_float_t)dir * dps;

    if (dir > 0) {
        while (m->position < target) {
            evo_motor_run_speed_control_c(m, target_dps);
            MICROPY_EVENT_POLL_HOOK;
            mp_hal_delay_ms(10);
        }
    } else {
        while (m->position > target) {
            evo_motor_run_speed_control_c(m, target_dps);
            MICROPY_EVENT_POLL_HOOK;
            mp_hal_delay_ms(10);
        }
    }

    evo_motor_run_speed_control_c(m, 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(evo_motor_runAngle_obj, evo_motor_runAngle);


static mp_obj_t evo_motor_getPosition(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_check(m);
    return mp_obj_new_int(m->position);
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_getPosition_obj, evo_motor_getPosition);

static mp_obj_t evo_motor_resetPosition(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_check(m);
    m->position = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_resetPosition_obj, evo_motor_resetPosition);

static mp_obj_t evo_motor_getAngle(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(evo_motor_get_angle_deg(m));
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_getAngle_obj, evo_motor_getAngle);

static mp_obj_t evo_motor_resetAngle(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_reset_angle(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_resetAngle_obj, evo_motor_resetAngle);

static mp_obj_t evo_motor_setCountsPerRotation(mp_obj_t self_in, mp_obj_t cpr_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_check(m);
    int cpr = mp_obj_get_int(cpr_in);
    if (cpr <= 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("counts must be > 0"));
    }
    m->cpr = cpr;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(evo_motor_setCountsPerRotation_obj, evo_motor_setCountsPerRotation);

static mp_obj_t evo_motor_flipEncoderDirection(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_check(m);
    m->enc_dir = (int8_t)(-m->enc_dir);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_flipEncoderDirection_obj, evo_motor_flipEncoderDirection);


static mp_obj_t evo_motor_updateSpeed(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_update_speed_c(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_updateSpeed_obj, evo_motor_updateSpeed);

static mp_obj_t evo_motor_getSpeed(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(evo_motor_get_speed_dps_c(m));
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_getSpeed_obj, evo_motor_getSpeed);

static mp_obj_t evo_motor_getSpeedCPS(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(evo_motor_get_speed_cps_c(m));
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_getSpeedCPS_obj, evo_motor_getSpeedCPS);

static mp_obj_t evo_motor_getSpeedDPS(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(evo_motor_get_speed_dps_c(m));
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_getSpeedDPS_obj, evo_motor_getSpeedDPS);

static mp_obj_t evo_motor_getPower(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_check(m);
    return mp_obj_new_int(m->speed_power);
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_getPower_obj, evo_motor_getPower);

static mp_obj_t evo_motor_runSpeed(mp_obj_t self_in, mp_obj_t target_dps_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    mp_float_t target_dps = mp_obj_get_float(target_dps_in);
    evo_motor_run_speed_control_c(m, target_dps);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(evo_motor_runSpeed_obj, evo_motor_runSpeed);

static mp_obj_t evo_motor_setSpeedPID(size_t n_args, const mp_obj_t *args) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(args[0]);
    evo_motor_set_speed_pid_c(
        m,
        mp_obj_get_float(args[1]),
        mp_obj_get_float(args[2]),
        mp_obj_get_float(args[3])
    );
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(evo_motor_setSpeedPID_obj, 4, 4, evo_motor_setSpeedPID);

static mp_obj_t evo_motor_setSpeedLimits(mp_obj_t self_in, mp_obj_t min_power_in, mp_obj_t max_power_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    int min_power = mp_obj_get_int(min_power_in);
    int max_power = mp_obj_get_int(max_power_in);

    evo_motor_set_speed_limits_c(m, min_power, max_power);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(evo_motor_setSpeedLimits_obj, evo_motor_setSpeedLimits);

static mp_obj_t evo_motor_resetSpeedControl(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_reset_speed_state(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_resetSpeedControl_obj, evo_motor_resetSpeedControl);

static mp_obj_t evo_motor_deinit(mp_obj_t self_in) {
    evo_motor_obj_t *m = MP_OBJ_TO_PTR(self_in);
    evo_motor_check(m);
    motor_stop_by_behaviour(m);
    detach_encoder_isr(m);
    m->pwm = NULL;
    m->valid = false;
    evo_motor_unregister(m);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_deinit_obj, evo_motor_deinit);

static mp_obj_t evo_motor_enableAllMotors(mp_obj_t self_in) {
    (void)self_in;
    evo_motor_enable_all();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_enableAllMotors_obj, evo_motor_enableAllMotors);

static mp_obj_t evo_motor_disableAllMotors(mp_obj_t self_in) {
    (void)self_in;
    evo_motor_disable_all();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motor_disableAllMotors_obj, evo_motor_disableAllMotors);

static const mp_rom_map_elem_t evo_motor_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_run),      MP_ROM_PTR(&evo_motor_run_obj) },
    { MP_ROM_QSTR(MP_QSTR_runPower), MP_ROM_PTR(&evo_motor_runPower_obj) },

    { MP_ROM_QSTR(MP_QSTR_setStopBehaviour), MP_ROM_PTR(&evo_motor_setStopBehaviour_obj) },
    { MP_ROM_QSTR(MP_QSTR_setStopBehavior),  MP_ROM_PTR(&evo_motor_setStopBehavior_obj)  },
    { MP_ROM_QSTR(MP_QSTR_brake), MP_ROM_PTR(&evo_motor_brake_obj) },
    { MP_ROM_QSTR(MP_QSTR_coast), MP_ROM_PTR(&evo_motor_coast_obj) },
    { MP_ROM_QSTR(MP_QSTR_hold),  MP_ROM_PTR(&evo_motor_hold_obj) },

    { MP_ROM_QSTR(MP_QSTR_runTime),  MP_ROM_PTR(&evo_motor_runTime_obj) },
    { MP_ROM_QSTR(MP_QSTR_runAngle), MP_ROM_PTR(&evo_motor_runAngle_obj) },

    { MP_ROM_QSTR(MP_QSTR_getPosition),   MP_ROM_PTR(&evo_motor_getPosition_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetPosition), MP_ROM_PTR(&evo_motor_resetPosition_obj) },
    { MP_ROM_QSTR(MP_QSTR_getAngle),      MP_ROM_PTR(&evo_motor_getAngle_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetAngle),    MP_ROM_PTR(&evo_motor_resetAngle_obj) },

    { MP_ROM_QSTR(MP_QSTR_setCountsPerRotation), MP_ROM_PTR(&evo_motor_setCountsPerRotation_obj) },
    { MP_ROM_QSTR(MP_QSTR_flipEncoderDirection), MP_ROM_PTR(&evo_motor_flipEncoderDirection_obj) },

    { MP_ROM_QSTR(MP_QSTR_updateSpeed),       MP_ROM_PTR(&evo_motor_updateSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_getSpeed),          MP_ROM_PTR(&evo_motor_getSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_getSpeedCPS),       MP_ROM_PTR(&evo_motor_getSpeedCPS_obj) },
    { MP_ROM_QSTR(MP_QSTR_getSpeedDPS),       MP_ROM_PTR(&evo_motor_getSpeedDPS_obj) },
    { MP_ROM_QSTR(MP_QSTR_getPower),          MP_ROM_PTR(&evo_motor_getPower_obj) },
    { MP_ROM_QSTR(MP_QSTR_runSpeed),          MP_ROM_PTR(&evo_motor_runSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_setSpeedPID),       MP_ROM_PTR(&evo_motor_setSpeedPID_obj) },
    { MP_ROM_QSTR(MP_QSTR_setSpeedLimits),    MP_ROM_PTR(&evo_motor_setSpeedLimits_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetSpeedControl), MP_ROM_PTR(&evo_motor_resetSpeedControl_obj) },

    { MP_ROM_QSTR(MP_QSTR_enableAllMotors),  MP_ROM_PTR(&evo_motor_enableAllMotors_obj) },
    { MP_ROM_QSTR(MP_QSTR_disableAllMotors), MP_ROM_PTR(&evo_motor_disableAllMotors_obj) },
    { MP_ROM_QSTR(MP_QSTR_enableAll),        MP_ROM_PTR(&evo_motor_enableAllMotors_obj) },
    { MP_ROM_QSTR(MP_QSTR_disableAll),       MP_ROM_PTR(&evo_motor_disableAllMotors_obj) },

    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&evo_motor_deinit_obj) },
};
static MP_DEFINE_CONST_DICT(evo_motor_locals_dict, evo_motor_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    evo_motor_type,
    MP_QSTR_EvoMotor,
    MP_TYPE_FLAG_NONE,
    make_new, evo_motor_make_new,
    locals_dict, &evo_motor_locals_dict
);
