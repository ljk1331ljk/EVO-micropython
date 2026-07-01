#ifndef EVO_MOTOR_H
#define EVO_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "py/obj.h"
#include "py/runtime.h"

#include "driver/gpio.h"

#include "evo_pwm.h"

// Port constants. If these are already defined in evo_board_pins.h or mod_evo.c,
// these guards prevent redefinition.
#ifndef EVO_M1
#define EVO_M1 1
#endif

#ifndef EVO_M2
#define EVO_M2 2
#endif

#ifndef EVO_M3
#define EVO_M3 3
#endif

#ifndef EVO_M4
#define EVO_M4 4
#endif

// Motor type constants.
#ifndef MT_GENERICWITHENCODER
#define MT_GENERICWITHENCODER 1
#endif

#ifndef MT_GENERICWITHOUTENCODER
#define MT_GENERICWITHOUTENCODER 2
#endif

#ifndef MT_EV3LargeMotor
#define MT_EV3LargeMotor 3
#endif

#ifndef MT_EV3MediumMotor
#define MT_EV3MediumMotor 4
#endif

#ifndef MT_GeekServoDCMotor
#define MT_GeekServoDCMotor 5
#endif

#ifndef MT_ITERMK495
#define MT_ITERMK495 6
#endif

#ifndef MT_ITERMK330
#define MT_ITERMK330 7
#endif

#ifndef MT_ITERMK195
#define MT_ITERMK195 8
#endif

#ifndef MT_EVOMotor300
#define MT_EVOMotor300 9
#endif

#ifndef MT_EVOMotor100
#define MT_EVOMotor100 10
#endif

#define EVO_STOP_COAST 0
#define EVO_STOP_BRAKE 1
#define EVO_STOP_HOLD  2

#ifndef EVO_PWM_MAX
#define EVO_PWM_MAX 4095
#endif

typedef struct _evo_motor_obj_t {
    mp_obj_base_t base;

    evo_pwm_obj_t *pwm;

    uint8_t power1;
    uint8_t power2;

    gpio_num_t t1;
    gpio_num_t t2;

    volatile int32_t position;
    volatile uint8_t last_state;

    int cpr;
    int8_t enc_dir;

    bool isr_attached;
    volatile bool active;

    uint8_t stop_behaviour;

    // Encoder speed measurement
    int32_t speed_last_position;
    uint32_t speed_last_ms;
    uint32_t speed_min_update_ms; // minimum interval between speed recalculations
    mp_float_t speed_dt;
    mp_float_t speed_cps;        // counts per second
    mp_float_t speed_dps;        // degrees per second

    // Legacy speed-control tuning fields retained for API compatibility.
    mp_float_t speed_kp;
    mp_float_t speed_ki;
    mp_float_t speed_kd;
    mp_float_t speed_integral;
    mp_float_t speed_integral_limit;
    mp_float_t speed_last_error;

    int speed_power;             // last raw PWM power command
    int speed_min_power;         // feed-forward power to overcome static friction
    int speed_max_power;         // output limit, normally EVO_PWM_MAX

    bool valid;
    struct _evo_motor_obj_t *next;
} evo_motor_obj_t;

extern const mp_obj_type_t evo_motor_type;

// Raw PWM-power control. This preserves the existing run(power) / runPower(power) behavior.
void evo_motor_run_power_c(evo_motor_obj_t *m, int power);

// Shared motor-driver sleep control.
// If pins.py defines NSLEEP_PIN, enable sets it HIGH and disable sets it LOW.
// If NSLEEP_PIN is not defined, these functions do nothing.
void evo_motor_enable_all(void);
void evo_motor_disable_all(void);

void evo_motor_brake_c(evo_motor_obj_t *m);
void evo_motor_coast_c(evo_motor_obj_t *m);
void evo_motor_hold_c(evo_motor_obj_t *m);
void evo_motor_set_stop_behaviour_c(evo_motor_obj_t *m, uint8_t beh);

void evo_motor_reset_angle(evo_motor_obj_t *m);
int32_t evo_motor_get_angle_deg(evo_motor_obj_t *m);

// Encoder speed measurement.
void evo_motor_update_speed_c(evo_motor_obj_t *m);
mp_float_t evo_motor_get_speed_cps_c(evo_motor_obj_t *m);
mp_float_t evo_motor_get_speed_dps_c(evo_motor_obj_t *m);

// Legacy compatibility helpers. Movement is raw PWM power, not closed-loop speed.
void evo_motor_set_speed_pid_c(evo_motor_obj_t *m, mp_float_t kp, mp_float_t ki, mp_float_t kd);
void evo_motor_set_speed_limits_c(evo_motor_obj_t *m, int min_power, int max_power);
void evo_motor_run_speed_control_c(evo_motor_obj_t *m, mp_float_t target_dps);

void evo_motor_deinit_all(void);

#endif // EVO_MOTOR_H
