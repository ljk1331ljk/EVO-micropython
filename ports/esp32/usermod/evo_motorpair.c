#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "evo_motorpair.h"

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define EVO_PAIR_LOOP_MS (1)
#ifndef EVO_ACCEL_NONE
#define EVO_ACCEL_NONE      (0)
#endif
#ifndef EVO_ACCEL_TRAPEZOID
#define EVO_ACCEL_TRAPEZOID (1)
#endif
#ifndef EVO_ACCEL_SCURVE
#define EVO_ACCEL_SCURVE    (2)
#endif

static inline int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int abs_i(int v) {
    return v < 0 ? -v : v;
}

static inline int motor_angle_int(evo_motor_obj_t *m) {
    return (int)evo_motor_get_angle_deg(m);
}

static inline float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float smoothstep_f(float x) {
    x = clamp_f(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static inline int lerp_i(int a, int b, float t) {
    t = clamp_f(t, 0.0f, 1.0f);
    return (int)((float)a + ((float)(b - a) * t) + 0.5f);
}

static inline int normalize_stop_behavior_int(int stop) {
    if (stop != EVO_STOP_COAST &&
        stop != EVO_STOP_BRAKE &&
        stop != EVO_STOP_HOLD) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid stop behavior"));
    }
    return stop;
}

static inline int normalize_stop_behavior_obj(mp_obj_t obj) {
    return normalize_stop_behavior_int(mp_obj_get_int(obj));
}

static int normalize_acceleration_profile(mp_obj_t obj) {
    int profile = mp_obj_get_int(obj);

    if (profile != EVO_ACCEL_NONE &&
        profile != EVO_ACCEL_TRAPEZOID &&
        profile != EVO_ACCEL_SCURVE) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid acceleration profile"));
    }

    return profile;
}

static void pair_apply_stop_now(evo_motorpair_obj_t *self, int stop) {
    if (stop == EVO_STOP_HOLD) {
        evo_motor_hold_c(self->m1);
        evo_motor_hold_c(self->m2);
    } else if (stop == EVO_STOP_BRAKE) {
        evo_motor_brake_c(self->m1);
        evo_motor_brake_c(self->m2);
    } else {
        evo_motor_coast_c(self->m1);
        evo_motor_coast_c(self->m2);
    }
}


typedef struct {
    int leftSpeed;
    int rightSpeed;
    int stopBehavior;

    int leftDir;
    int rightDir;
    int maxSpeed;
    float leftPowerRatio;
    float rightPowerRatio;
    float leftPowerRatioInv;
    float rightPowerRatioInv;

    int encPError;
    int integralError[EVO_PAIR_INTEGRAL_BUF_SIZE];
    int integralIndex;
    int integralSum;

    int speed;
    int enc;
    int encError;
    int lSpeed;
    int rSpeed;

    int degrees;
    int accel;
    int decel;
    int accelDist;
    int decelDist;
    int startSpeed;
    int endSpeed;

    int timems;
    int slowdowntime;
    int accelTimeMs;
    uint32_t t0_ms;
} evo_pair_exec_t;

static void pair_prepare_common(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    st->leftDir  = (st->leftSpeed  == 0) ? 0 : (st->leftSpeed  > 0 ? 1 : -1);
    st->rightDir = (st->rightSpeed == 0) ? 0 : (st->rightSpeed > 0 ? 1 : -1);
    st->maxSpeed = MAX(st->leftSpeed * st->leftDir, st->rightSpeed * st->rightDir);

    if (abs_i(st->leftSpeed) == abs_i(st->rightSpeed)) {
        st->leftPowerRatio = 1.0f;
        st->rightPowerRatio = 1.0f;
    } else if (st->leftSpeed != 0 && st->rightSpeed != 0) {
        if (abs_i(st->leftSpeed) > abs_i(st->rightSpeed)) {
            st->leftPowerRatio = 1.0f;
            st->rightPowerRatio = fabsf((float)st->rightSpeed / (float)st->leftSpeed);
        } else {
            st->rightPowerRatio = 1.0f;
            st->leftPowerRatio = fabsf((float)st->leftSpeed / (float)st->rightSpeed);
        }
    } else {
        st->leftPowerRatio  = (st->leftSpeed == 0) ? 0.0f : 1.0f;
        st->rightPowerRatio = (st->rightSpeed == 0) ? 0.0f : 1.0f;
    }

    st->leftPowerRatioInv = (st->leftPowerRatio == 0.0f) ? 0.0f : (1.0f / st->leftPowerRatio);
    st->rightPowerRatioInv = (st->rightPowerRatio == 0.0f) ? 0.0f : (1.0f / st->rightPowerRatio);

    st->encPError = 0;
    memset(st->integralError, 0, sizeof(st->integralError));
    st->integralIndex = 0;
    st->integralSum = 0;
    st->enc = 0;
    st->encError = 0;
    st->lSpeed = 0;
    st->rSpeed = 0;
    st->accelTimeMs = 0;

    (void)self;
}

static int pair_calc_degrees_profile_speed(evo_motorpair_obj_t *self, evo_pair_exec_t *st, int progress) {
    if (st->maxSpeed <= 0) {
        return 0;
    }

    if (self->accelerationProfile == EVO_ACCEL_NONE || st->degrees <= 0) {
        return st->maxSpeed;
    }

    progress = clamp_i(progress, 0, st->degrees);

    if (progress < (st->degrees - st->decelDist)) {
        if (st->accelDist <= 0 || progress >= st->accelDist || st->accel <= 0) {
            return st->maxSpeed;
        }

        if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
            float t = smoothstep_f((float)progress / (float)st->accelDist);
            return clamp_i(lerp_i(st->startSpeed, st->maxSpeed, t), st->startSpeed, st->maxSpeed);
        }

        return clamp_i(
            st->startSpeed + ((st->maxSpeed - st->startSpeed) * progress) / st->accelDist,
            st->startSpeed,
            st->maxSpeed
        );
    }

    if (st->decelDist <= 0 || st->decel <= 0) {
        return st->endSpeed;
    }

    int decelProgress = progress - (st->degrees - st->decelDist);
    decelProgress = clamp_i(decelProgress, 0, st->decelDist);

    if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
        float t = smoothstep_f((float)decelProgress / (float)st->decelDist);
        return clamp_i(lerp_i(st->maxSpeed, st->endSpeed, t), st->endSpeed, st->maxSpeed);
    }

    return clamp_i(
        st->maxSpeed - ((st->maxSpeed - st->endSpeed) * decelProgress) / st->decelDist,
        st->endSpeed,
        st->maxSpeed
    );
}

static int pair_calc_time_profile_speed(evo_motorpair_obj_t *self, evo_pair_exec_t *st, int elapsed) {
    if (st->maxSpeed <= 0) {
        return 0;
    }

    if (self->accelerationProfile == EVO_ACCEL_NONE) {
        return st->maxSpeed;
    }

    elapsed = clamp_i(elapsed, 0, st->timems);

    if (st->accelTimeMs > 0 && elapsed < st->accelTimeMs) {
        float t = (float)elapsed / (float)st->accelTimeMs;
        if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
            t = smoothstep_f(t);
        }
        return clamp_i(lerp_i(st->startSpeed, st->maxSpeed, t), st->startSpeed, st->maxSpeed);
    }

    int decelStart = st->timems - st->slowdowntime;
    if (st->slowdowntime > 0 && elapsed >= decelStart) {
        float t = (float)(elapsed - decelStart) / (float)st->slowdowntime;
        if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
            t = smoothstep_f(t);
        }
        return clamp_i(lerp_i(st->maxSpeed, st->endSpeed, t), st->endSpeed, st->maxSpeed);
    }

    return st->maxSpeed;
}

static void pair_apply_speed_and_sync(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    if (st->leftSpeed != 0 && st->rightSpeed != 0) {
        int sync = st->encError * self->kpSync
                 + (st->encError - st->encPError) * self->kdSync
                 + st->integralSum * self->kiSync;

        st->lSpeed = (int)((st->speed * st->leftPowerRatio) - sync) * st->leftDir;
        st->rSpeed = (int)((st->speed * st->rightPowerRatio) + sync) * st->rightDir;
    } else if (st->leftSpeed == 0) {
        st->lSpeed = 0;
        st->rSpeed = (int)(st->speed * st->rightPowerRatio) * st->rightDir;
    } else {
        st->rSpeed = 0;
        st->lSpeed = (int)(st->speed * st->leftPowerRatio) * st->leftDir;
    }

    evo_motor_run_speed_control_c(self->m1, (mp_float_t)st->lSpeed);
    evo_motor_run_speed_control_c(self->m2, (mp_float_t)st->rSpeed);
}

static void pair_update_encoder_state(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    int a1 = motor_angle_int(self->m1);
    int a2 = motor_angle_int(self->m2);

    int leftEnc  = (st->leftPowerRatioInv == 0.0f) ? 0 : (int)((float)a1 * st->leftPowerRatioInv) * st->leftDir;
    int rightEnc = (st->rightPowerRatioInv == 0.0f) ? 0 : (int)((float)a2 * st->rightPowerRatioInv) * st->rightDir;

    if (leftEnc < 0) { leftEnc = -leftEnc; }
    if (rightEnc < 0) { rightEnc = -rightEnc; }

    if (st->leftSpeed != 0 && st->rightSpeed != 0) {
        st->enc = (leftEnc + rightEnc) / 2;
        st->encError = leftEnc - rightEnc;
    } else if (st->leftSpeed == 0) {
        st->enc = rightEnc;
        st->encError = 0;
    } else {
        st->enc = leftEnc;
        st->encError = 0;
    }

    int evictionIndex = st->integralIndex;
    st->integralSum += st->encError - st->integralError[evictionIndex];
    st->integralError[evictionIndex] = st->encError;
}

static void pair_init_move_degrees(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    evo_motor_reset_angle(self->m1);
    evo_motor_reset_angle(self->m2);

    self->m1->speed_integral = 0;
    self->m1->speed_last_error = 0;
    self->m1->speed_last_ms = 0;
    self->m2->speed_integral = 0;
    self->m2->speed_last_error = 0;
    self->m2->speed_last_ms = 0;

    pair_prepare_common(self, st);

    st->startSpeed = MIN(self->startSpeed, st->maxSpeed);
    st->endSpeed = MIN(self->endSpeed, st->maxSpeed);

    if (self->accelerationProfile == EVO_ACCEL_NONE) {
        st->accel = 0;
        st->decel = 0;
        st->accelDist = 0;
        st->decelDist = 0;
        st->startSpeed = st->maxSpeed;
        st->endSpeed = st->maxSpeed;
    } else {
        st->accel = (st->leftSpeed == 0 || st->rightSpeed == 0) ? (self->accel * 2) : self->accel;
        st->decel = (st->leftSpeed == 0 || st->rightSpeed == 0) ? (self->decel * 2) : self->decel;

        if (st->accel <= 0 || st->maxSpeed <= st->startSpeed) {
            st->accel = 0;
            st->accelDist = 0;
            st->startSpeed = st->maxSpeed;
        } else {
            st->accelDist = (st->maxSpeed * st->maxSpeed - st->startSpeed * st->startSpeed) / 2 / st->accel;
        }

        if (st->decel <= 0 || st->maxSpeed <= st->endSpeed) {
            st->decel = 0;
            st->decelDist = 0;
            st->endSpeed = st->maxSpeed;
        } else {
            st->decelDist = (st->maxSpeed * st->maxSpeed - st->endSpeed * st->endSpeed) / 2 / st->decel;
        }

        if (st->accelDist + st->decelDist > st->degrees && (st->accel + st->decel) > 0) {
            st->decelDist = ((st->startSpeed * st->startSpeed - st->endSpeed * st->endSpeed) / 2 + st->accel * st->degrees) / (st->accel + st->decel);
            st->decelDist = clamp_i(st->decelDist, 0, st->degrees);
            st->accelDist = st->degrees - st->decelDist;
            st->maxSpeed = (int)sqrtf((float)(st->startSpeed * st->startSpeed + 2 * st->accel * st->accelDist));
        }
    }

    st->speed = st->startSpeed;
    pair_update_encoder_state(self, st);
}

static bool pair_step_move_degrees(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    pair_update_encoder_state(self, st);

    if (st->enc >= st->degrees) {
        pair_apply_stop_now(self, st->stopBehavior);
        return true;
    }

    st->speed = pair_calc_degrees_profile_speed(self, st, st->enc);
    pair_apply_speed_and_sync(self, st);

    st->integralIndex++;
    if (st->integralIndex >= EVO_PAIR_INTEGRAL_BUF_SIZE) {
        st->integralIndex = 0;
    }
    st->encPError = st->encError;
    return false;
}

static void pair_run_move_degrees(evo_motorpair_obj_t *self,
                                  int leftSpeed,
                                  int rightSpeed,
                                  int degrees,
                                  int stopBehavior) {
    evo_pair_exec_t st;
    st.leftSpeed = leftSpeed;
    st.rightSpeed = rightSpeed;
    st.degrees = abs_i(degrees);
    st.stopBehavior = stopBehavior;

    self->busy = true;
    pair_init_move_degrees(self, &st);

    while (1) {
        MICROPY_EVENT_POLL_HOOK;
        if (pair_step_move_degrees(self, &st)) {
            break;
        }
        mp_hal_delay_ms(EVO_PAIR_LOOP_MS);
    }
    self->busy = false;
}

static void pair_init_move_time(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    evo_motor_reset_angle(self->m1);
    evo_motor_reset_angle(self->m2);

    self->m1->speed_integral = 0;
    self->m1->speed_last_error = 0;
    self->m1->speed_last_ms = 0;
    self->m2->speed_integral = 0;
    self->m2->speed_last_error = 0;
    self->m2->speed_last_ms = 0;

    pair_prepare_common(self, st);

    st->startSpeed = MIN(self->startSpeed, st->maxSpeed);
    st->endSpeed = MIN(self->endSpeed, st->maxSpeed);

    st->accel = (st->leftSpeed == 0 || st->rightSpeed == 0) ? (self->accel * 2) : self->accel;
    st->decel = (st->leftSpeed == 0 || st->rightSpeed == 0) ? (self->decel * 2) : self->decel;

    if (self->accelerationProfile == EVO_ACCEL_NONE || st->accel <= 0 || st->maxSpeed <= st->startSpeed) {
        st->accelTimeMs = 0;
        st->startSpeed = st->maxSpeed;
    } else {
        st->accelTimeMs = (int)((float)(st->maxSpeed - st->startSpeed) * 1000.0f / (float)st->accel);
        if (st->accelTimeMs < 0) {
            st->accelTimeMs = 0;
        }
    }

    if (st->slowdowntime < 0) {
        st->slowdowntime = 0;
    }
    if (st->slowdowntime > st->timems) {
        st->slowdowntime = st->timems;
    }
    if (st->accelTimeMs > st->timems - st->slowdowntime) {
        st->accelTimeMs = MAX(0, st->timems - st->slowdowntime);
    }

    st->speed = st->startSpeed;
    st->t0_ms = mp_hal_ticks_ms();
}

static bool pair_step_move_time(evo_motorpair_obj_t *self, evo_pair_exec_t *st) {
    int elapsed = (int)((uint32_t)mp_hal_ticks_ms() - st->t0_ms);

    if (elapsed >= st->timems) {
        pair_apply_stop_now(self, st->stopBehavior);
        return true;
    }

    pair_update_encoder_state(self, st);

    st->speed = pair_calc_time_profile_speed(self, st, elapsed);
    pair_apply_speed_and_sync(self, st);

    st->integralIndex++;
    if (st->integralIndex >= EVO_PAIR_INTEGRAL_BUF_SIZE) {
        st->integralIndex = 0;
    }
    st->encPError = st->encError;
    return false;
}

static void pair_run_move_time(evo_motorpair_obj_t *self,
                               int leftSpeed,
                               int rightSpeed,
                               int timems,
                               int slowdowntime,
                               int stopBehavior) {
    evo_pair_exec_t st;
    st.leftSpeed = leftSpeed;
    st.rightSpeed = rightSpeed;
    st.timems = timems;
    st.slowdowntime = slowdowntime;
    st.stopBehavior = stopBehavior;

    self->busy = true;
    pair_init_move_time(self, &st);

    while (1) {
        MICROPY_EVENT_POLL_HOOK;
        if (pair_step_move_time(self, &st)) {
            break;
        }
        mp_hal_delay_ms(EVO_PAIR_LOOP_MS);
    }
    self->busy = false;
}

static mp_obj_t evo_motorpair_make_new(const mp_obj_type_t *type,
                                       size_t n_args,
                                       size_t n_kw,
                                       const mp_obj_t *all_args) {
    enum { ARG_m1, ARG_m2 };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_m1, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_m2, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    evo_motorpair_obj_t *self = mp_obj_malloc(evo_motorpair_obj_t, type);

    self->m1 = (evo_motor_obj_t *)MP_OBJ_TO_PTR(args[ARG_m1].u_obj);
    self->m2 = (evo_motor_obj_t *)MP_OBJ_TO_PTR(args[ARG_m2].u_obj);

    self->startSpeed = 800;
    self->endSpeed = 800;
    self->accel = 10000;
    self->decel = 10000;
    self->accelerationProfile = EVO_ACCEL_TRAPEZOID;

    self->kpSync = 70;
    self->kiSync = 5;
    self->kdSync = 800;

    self->stopBehavior = EVO_STOP_BRAKE;
    self->busy = false;

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t mp_setStartSpeed(mp_obj_t self_in, mp_obj_t v_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->startSpeed = abs_i(mp_obj_get_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setStartSpeed_obj, mp_setStartSpeed);

static mp_obj_t mp_setEndSpeed(mp_obj_t self_in, mp_obj_t v_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->endSpeed = abs_i(mp_obj_get_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setEndSpeed_obj, mp_setEndSpeed);

static mp_obj_t mp_setAcceleration(mp_obj_t self_in, mp_obj_t v_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->accel = abs_i(mp_obj_get_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setAcceleration_obj, mp_setAcceleration);

static mp_obj_t mp_getAcceleration(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->accel);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getAcceleration_obj, mp_getAcceleration);

static mp_obj_t mp_setDeceleration(mp_obj_t self_in, mp_obj_t v_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->decel = abs_i(mp_obj_get_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setDeceleration_obj, mp_setDeceleration);

static mp_obj_t mp_getDeceleration(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->decel);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getDeceleration_obj, mp_getDeceleration);

static mp_obj_t mp_setAccelerationProfile(mp_obj_t self_in, mp_obj_t profile_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->accelerationProfile = (uint8_t)normalize_acceleration_profile(profile_in);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setAccelerationProfile_obj, mp_setAccelerationProfile);

static mp_obj_t mp_getAccelerationProfile(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->accelerationProfile);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getAccelerationProfile_obj, mp_getAccelerationProfile);

static mp_obj_t mp_setStopBehaviour(mp_obj_t self_in, mp_obj_t stop_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->stopBehavior = (uint8_t)normalize_stop_behavior_obj(stop_in);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setStopBehaviour_obj, mp_setStopBehaviour);

static mp_obj_t mp_setStopBehavior(mp_obj_t self_in, mp_obj_t stop_in) {
    return mp_setStopBehaviour(self_in, stop_in);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setStopBehavior_obj, mp_setStopBehavior);

static mp_obj_t mp_setSyncPID(size_t n_args, const mp_obj_t *args) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->kpSync = mp_obj_get_int(args[1]);
    self->kiSync = mp_obj_get_int(args[2]);
    self->kdSync = mp_obj_get_int(args[3]);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_setSyncPID_obj, 4, 4, mp_setSyncPID);

static mp_obj_t mp_getSyncPID(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_obj_t items[3];
    items[0] = mp_obj_new_int(self->kpSync);
    items[1] = mp_obj_new_int(self->kiSync);
    items[2] = mp_obj_new_int(self->kdSync);

    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getSyncPID_obj, mp_getSyncPID);

static mp_obj_t mp_movePower(size_t n_args, const mp_obj_t *args) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->busy = false;
    evo_motor_run_power_c(self->m1, mp_obj_get_int(args[1]));
    evo_motor_run_power_c(self->m2, mp_obj_get_int(args[2]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_movePower_obj, 3, 3, mp_movePower);

static mp_obj_t mp_moveSpeed(size_t n_args, const mp_obj_t *args) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->busy = false;
    evo_motor_run_speed_control_c(self->m1, (mp_float_t)mp_obj_get_float(args[1]));
    evo_motor_run_speed_control_c(self->m2, (mp_float_t)mp_obj_get_float(args[2]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_moveSpeed_obj, 3, 3, mp_moveSpeed);

// Backward-compatible alias. move() is speed-based DPS control.
static mp_obj_t mp_move(size_t n_args, const mp_obj_t *args) {
    return mp_moveSpeed(n_args, args);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_move_obj, 3, 3, mp_move);

// moveDegrees/runDegrees(leftSpeed, rightSpeed, degrees[, stopBehavior])
// The encoder synchronization keeps both motors following the requested speed ratio.
static mp_obj_t mp_moveDegrees(size_t n_args, const mp_obj_t *args) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int leftSpeed = mp_obj_get_int(args[1]);
    int rightSpeed = mp_obj_get_int(args[2]);
    int degrees = mp_obj_get_int(args[3]);
    int stopBehavior = (n_args >= 5)
        ? normalize_stop_behavior_obj(args[4])
        : self->stopBehavior;

    pair_run_move_degrees(self, leftSpeed, rightSpeed, degrees, stopBehavior);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_moveDegrees_obj, 4, 5, mp_moveDegrees);


static mp_obj_t mp_moveTime(size_t n_args, const mp_obj_t *args) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int leftSpeed = mp_obj_get_int(args[1]);
    int rightSpeed = mp_obj_get_int(args[2]);
    int timems = mp_obj_get_int(args[3]);
    int slowdowntime = (n_args >= 5) ? mp_obj_get_int(args[4]) : 200;
    int stopBehavior = (n_args >= 6)
        ? normalize_stop_behavior_obj(args[5])
        : self->stopBehavior;

    if (timems < 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("time must be >= 0"));
    }

    pair_run_move_time(self, leftSpeed, rightSpeed, timems, slowdowntime, stopBehavior);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_moveTime_obj, 4, 6, mp_moveTime);


static mp_obj_t mp_hold(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    evo_motor_hold_c(self->m1);
    evo_motor_hold_c(self->m2);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hold_obj, mp_hold);

static mp_obj_t mp_brake(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    evo_motor_brake_c(self->m1);
    evo_motor_brake_c(self->m2);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_brake_obj, mp_brake);

static mp_obj_t mp_coast(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    evo_motor_coast_c(self->m1);
    evo_motor_coast_c(self->m2);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_coast_obj, mp_coast);

static mp_obj_t mp_stop(size_t n_args, const mp_obj_t *args) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int stopBehavior = (n_args >= 2)
        ? normalize_stop_behavior_obj(args[1])
        : EVO_STOP_BRAKE;

    self->busy = false;
    pair_apply_stop_now(self, stopBehavior);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_stop_obj, 1, 2, mp_stop);

static mp_obj_t mp_resetAngle(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    evo_motor_reset_angle(self->m1);
    evo_motor_reset_angle(self->m2);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_resetAngle_obj, mp_resetAngle);

static mp_obj_t mp_isBusy(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->busy);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_isBusy_obj, mp_isBusy);


static mp_obj_t evo_motorpair_deinit_method(mp_obj_t self_in) {
    evo_motorpair_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    pair_apply_stop_now(self, EVO_STOP_BRAKE);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_motorpair_deinit_method_obj, evo_motorpair_deinit_method);

static const mp_rom_map_elem_t evo_motorpair_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_setStartSpeed),          MP_ROM_PTR(&mp_setStartSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_setEndSpeed),            MP_ROM_PTR(&mp_setEndSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_setAcceleration),        MP_ROM_PTR(&mp_setAcceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_getAcceleration),        MP_ROM_PTR(&mp_getAcceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_setDeceleration),        MP_ROM_PTR(&mp_setDeceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_getDeceleration),        MP_ROM_PTR(&mp_getDeceleration_obj) },

    { MP_ROM_QSTR(MP_QSTR_setAccelerationProfile), MP_ROM_PTR(&mp_setAccelerationProfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_getAccelerationProfile), MP_ROM_PTR(&mp_getAccelerationProfile_obj) },

    { MP_ROM_QSTR(MP_QSTR_setStopBehaviour),       MP_ROM_PTR(&mp_setStopBehaviour_obj) },
    { MP_ROM_QSTR(MP_QSTR_setStopBehavior),        MP_ROM_PTR(&mp_setStopBehavior_obj) },

    { MP_ROM_QSTR(MP_QSTR_setSyncPID),             MP_ROM_PTR(&mp_setSyncPID_obj) },
    { MP_ROM_QSTR(MP_QSTR_getSyncPID),             MP_ROM_PTR(&mp_getSyncPID_obj) },

    { MP_ROM_QSTR(MP_QSTR_move),                   MP_ROM_PTR(&mp_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_moveSpeed),              MP_ROM_PTR(&mp_moveSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_movePower),              MP_ROM_PTR(&mp_movePower_obj) },
    { MP_ROM_QSTR(MP_QSTR_moveDegrees),            MP_ROM_PTR(&mp_moveDegrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_runDegrees),             MP_ROM_PTR(&mp_moveDegrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_moveTime),               MP_ROM_PTR(&mp_moveTime_obj) },

    { MP_ROM_QSTR(MP_QSTR_hold),                   MP_ROM_PTR(&mp_hold_obj) },
    { MP_ROM_QSTR(MP_QSTR_brake),                  MP_ROM_PTR(&mp_brake_obj) },
    { MP_ROM_QSTR(MP_QSTR_coast),                  MP_ROM_PTR(&mp_coast_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),                   MP_ROM_PTR(&mp_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetAngle),             MP_ROM_PTR(&mp_resetAngle_obj) },

    { MP_ROM_QSTR(MP_QSTR_isBusy),                 MP_ROM_PTR(&mp_isBusy_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),                 MP_ROM_PTR(&evo_motorpair_deinit_method_obj) },
};
static MP_DEFINE_CONST_DICT(evo_motorpair_locals_dict, evo_motorpair_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    evo_motorpair_type,
    MP_QSTR_EvoMotorPair,
    MP_TYPE_FLAG_NONE,
    make_new, evo_motorpair_make_new,
    locals_dict, &evo_motorpair_locals_dict
);
