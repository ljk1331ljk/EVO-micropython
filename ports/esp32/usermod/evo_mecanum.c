#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "py/runtime.h"
#include "py/mphal.h"
#include "evo_mecanum.h"

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define EVO_MECANUM_LOOP_MS (1)

// Encoder synchronisation PID.
// Error is measured in virtual wheel degrees, where each wheel's encoder
// movement is divided by its mecanum ratio. This lets all active wheels
// track the same movement progress even when their target wheel speeds differ.
#define EVO_MECANUM_SYNC_KP_DEFAULT (4.0f)
#define EVO_MECANUM_SYNC_KI_DEFAULT (0.0f)
#define EVO_MECANUM_SYNC_KD_DEFAULT (0.15f)
#define EVO_MECANUM_SYNC_INTEGRAL_LIMIT (250.0f)
#define EVO_MECANUM_SYNC_CORRECTION_LIMIT_RATIO (0.60f)

static inline int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline int abs_i(int v) {
    return v < 0 ? -v : v;
}

static inline int obj_get_rounded_int(mp_obj_t obj) {
    return (int)roundf((float)mp_obj_get_float(obj));
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

static void mecanum_apply_stop_now(evo_mecanum_obj_t *self, int stop) {
    if (stop == EVO_STOP_HOLD) {
        evo_motor_hold_c(self->frontLeft);
        evo_motor_hold_c(self->frontRight);
        evo_motor_hold_c(self->rearLeft);
        evo_motor_hold_c(self->rearRight);
    } else if (stop == EVO_STOP_BRAKE) {
        evo_motor_brake_c(self->frontLeft);
        evo_motor_brake_c(self->frontRight);
        evo_motor_brake_c(self->rearLeft);
        evo_motor_brake_c(self->rearRight);
    } else {
        evo_motor_coast_c(self->frontLeft);
        evo_motor_coast_c(self->frontRight);
        evo_motor_coast_c(self->rearLeft);
        evo_motor_coast_c(self->rearRight);
    }
}

typedef struct {
    int xSpeed;
    int ySpeed;
    int turnSpeed;
    int stopBehavior;

    int flTarget;
    int frTarget;
    int rlTarget;
    int rrTarget;

    int flDir;
    int frDir;
    int rlDir;
    int rrDir;

    float flRatio;
    float frRatio;
    float rlRatio;
    float rrRatio;

    int maxSpeed;
    int currentSpeed;

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

    int flStart;
    int frStart;
    int rlStart;
    int rrStart;

    float flErrPrev;
    float frErrPrev;
    float rlErrPrev;
    float rrErrPrev;

    float flErrInt;
    float frErrInt;
    float rlErrInt;
    float rrErrInt;

    uint32_t syncLastMs;
} evo_mecanum_exec_t;

static void mecanum_calc_wheel_targets(evo_mecanum_exec_t *st) {
    // Coordinate convention:
    // xSpeed: strafe right positive
    // ySpeed: forward positive
    // turnSpeed: clockwise positive
    int fl = st->ySpeed + st->xSpeed + st->turnSpeed;
    int fr = st->ySpeed - st->xSpeed - st->turnSpeed;
    int rl = st->ySpeed - st->xSpeed + st->turnSpeed;
    int rr = st->ySpeed + st->xSpeed - st->turnSpeed;

    int maxAbs = MAX(MAX(abs_i(fl), abs_i(fr)), MAX(abs_i(rl), abs_i(rr)));
    int inputMax = MAX(MAX(abs_i(st->xSpeed), abs_i(st->ySpeed)), abs_i(st->turnSpeed));

    if (maxAbs > 0 && inputMax > 0 && maxAbs > inputMax) {
        fl = (fl * inputMax) / maxAbs;
        fr = (fr * inputMax) / maxAbs;
        rl = (rl * inputMax) / maxAbs;
        rr = (rr * inputMax) / maxAbs;
    }

    st->flTarget = fl;
    st->frTarget = fr;
    st->rlTarget = rl;
    st->rrTarget = rr;
}

static void mecanum_prepare_common(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    mecanum_calc_wheel_targets(st);

    st->flDir = (st->flTarget == 0) ? 0 : (st->flTarget > 0 ? 1 : -1);
    st->frDir = (st->frTarget == 0) ? 0 : (st->frTarget > 0 ? 1 : -1);
    st->rlDir = (st->rlTarget == 0) ? 0 : (st->rlTarget > 0 ? 1 : -1);
    st->rrDir = (st->rrTarget == 0) ? 0 : (st->rrTarget > 0 ? 1 : -1);

    st->maxSpeed = MAX(MAX(abs_i(st->flTarget), abs_i(st->frTarget)),
                      MAX(abs_i(st->rlTarget), abs_i(st->rrTarget)));

    if (st->maxSpeed > 0) {
        st->flRatio = (float)abs_i(st->flTarget) / (float)st->maxSpeed;
        st->frRatio = (float)abs_i(st->frTarget) / (float)st->maxSpeed;
        st->rlRatio = (float)abs_i(st->rlTarget) / (float)st->maxSpeed;
        st->rrRatio = (float)abs_i(st->rrTarget) / (float)st->maxSpeed;
    } else {
        st->flRatio = 0.0f;
        st->frRatio = 0.0f;
        st->rlRatio = 0.0f;
        st->rrRatio = 0.0f;
    }

    st->accel = abs_i(self->accel);
    st->decel = abs_i(self->decel);
    st->startSpeed = MIN(abs_i(self->startSpeed), st->maxSpeed);
    st->endSpeed = MIN(abs_i(self->endSpeed), st->maxSpeed);
    st->currentSpeed = 0;
}

static void mecanum_get_abs_angles(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st,
                                   int *fl, int *fr, int *rl, int *rr) {
    *fl = abs_i((int)evo_motor_get_angle_deg(self->frontLeft) - st->flStart);
    *fr = abs_i((int)evo_motor_get_angle_deg(self->frontRight) - st->frStart);
    *rl = abs_i((int)evo_motor_get_angle_deg(self->rearLeft) - st->rlStart);
    *rr = abs_i((int)evo_motor_get_angle_deg(self->rearRight) - st->rrStart);
}

static float mecanum_virtual_progress_for_wheel(int angle, float ratio) {
    if (ratio <= 0.0f) {
        return 0.0f;
    }
    return (float)angle / ratio;
}

static int mecanum_angle_progress(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    int fl, fr, rl, rr;
    mecanum_get_abs_angles(self, st, &fl, &fr, &rl, &rr);

    float sum = 0.0f;
    int count = 0;
    if (st->flRatio > 0.0f) { sum += mecanum_virtual_progress_for_wheel(fl, st->flRatio); count++; }
    if (st->frRatio > 0.0f) { sum += mecanum_virtual_progress_for_wheel(fr, st->frRatio); count++; }
    if (st->rlRatio > 0.0f) { sum += mecanum_virtual_progress_for_wheel(rl, st->rlRatio); count++; }
    if (st->rrRatio > 0.0f) { sum += mecanum_virtual_progress_for_wheel(rr, st->rrRatio); count++; }

    return (count == 0) ? 0 : (int)(sum / (float)count);
}

static int mecanum_target_degrees_for_ratio(int degrees, float ratio) {
    if (ratio <= 0.0f || degrees <= 0) {
        return 0;
    }
    int target = (int)(((float)degrees * ratio) + 0.5f);
    return MAX(1, target);
}

static bool mecanum_degrees_complete(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    int fl, fr, rl, rr;
    mecanum_get_abs_angles(self, st, &fl, &fr, &rl, &rr);

    if (st->flRatio > 0.0f && fl < mecanum_target_degrees_for_ratio(st->degrees, st->flRatio)) { return false; }
    if (st->frRatio > 0.0f && fr < mecanum_target_degrees_for_ratio(st->degrees, st->frRatio)) { return false; }
    if (st->rlRatio > 0.0f && rl < mecanum_target_degrees_for_ratio(st->degrees, st->rlRatio)) { return false; }
    if (st->rrRatio > 0.0f && rr < mecanum_target_degrees_for_ratio(st->degrees, st->rrRatio)) { return false; }
    return true;
}

static float mecanum_pid_step(evo_mecanum_obj_t *self,
                              float err,
                              float *prev,
                              float *integ,
                              float dt_s,
                              float correction_limit) {
    *integ += err * dt_s;
    *integ = clamp_f(*integ, -EVO_MECANUM_SYNC_INTEGRAL_LIMIT, EVO_MECANUM_SYNC_INTEGRAL_LIMIT);

    float deriv = (dt_s > 0.0f) ? ((err - *prev) / dt_s) : 0.0f;
    *prev = err;

    float out = (self->kpSync * err) +
                (self->kiSync * (*integ)) +
                (self->kdSync * deriv);

    return clamp_f(out, -correction_limit, correction_limit);
}

static int mecanum_apply_pid_to_wheel(evo_mecanum_obj_t *self,
                                      int base_abs_speed,
                                      int dir,
                                      float ratio,
                                      float ref_progress,
                                      float wheel_progress,
                                      float *prev,
                                      float *integ,
                                      float dt_s,
                                      int correction_limit,
                                      int maxSpeed) {
    if (ratio <= 0.0f || dir == 0 || base_abs_speed <= 0) {
        *prev = 0.0f;
        *integ = 0.0f;
        return 0;
    }

    // Positive error means this wheel is behind the shared virtual progress.
    float err = ref_progress - wheel_progress;
    float correction = mecanum_pid_step(self, err, prev, integ, dt_s, (float)correction_limit);
    int corrected_abs = base_abs_speed + (int)correction;
    corrected_abs = clamp_i(corrected_abs, 0, maxSpeed);
    return corrected_abs * dir;
}

static int mecanum_calc_degrees_profile_speed(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st, int progress) {
    if (st->maxSpeed <= 0) {
        return 0;
    }
    if (self->accelerationProfile == EVO_ACCEL_NONE || st->degrees <= 0) {
        return st->maxSpeed;
    }

    progress = clamp_i(progress, 0, st->degrees);
    int target = st->maxSpeed;

    if (st->accelDist > 0 && progress < st->accelDist) {
        float t = (float)progress / (float)st->accelDist;
        if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
            t = smoothstep_f(t);
        }
        target = lerp_i(st->startSpeed, st->maxSpeed, t);
    }

    int remaining = st->degrees - progress;
    if (st->decelDist > 0 && remaining < st->decelDist) {
        float t = (float)remaining / (float)st->decelDist;
        if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
            t = smoothstep_f(t);
        }
        int decelTarget = lerp_i(st->endSpeed, st->maxSpeed, t);
        target = MIN(target, decelTarget);
    }

    return clamp_i(target, 0, st->maxSpeed);
}

static int mecanum_calc_time_profile_speed(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st, uint32_t elapsed) {
    if (st->maxSpeed <= 0) {
        return 0;
    }
    if (self->accelerationProfile == EVO_ACCEL_NONE || st->timems <= 0) {
        return st->maxSpeed;
    }

    int target = st->maxSpeed;
    if (st->accelTimeMs > 0 && elapsed < (uint32_t)st->accelTimeMs) {
        float t = (float)elapsed / (float)st->accelTimeMs;
        if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
            t = smoothstep_f(t);
        }
        target = lerp_i(st->startSpeed, st->maxSpeed, t);
    }

    if (st->slowdowntime > 0 && elapsed < (uint32_t)st->timems) {
        uint32_t remaining = (uint32_t)st->timems - elapsed;
        if (remaining < (uint32_t)st->slowdowntime) {
            float t = (float)remaining / (float)st->slowdowntime;
            if (self->accelerationProfile == EVO_ACCEL_SCURVE) {
                t = smoothstep_f(t);
            }
            int decelTarget = lerp_i(st->endSpeed, st->maxSpeed, t);
            target = MIN(target, decelTarget);
        }
    }

    return clamp_i(target, 0, st->maxSpeed);
}

static void mecanum_apply_speed(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    int fl = (int)((float)st->currentSpeed * st->flRatio) * st->flDir;
    int fr = (int)((float)st->currentSpeed * st->frRatio) * st->frDir;
    int rl = (int)((float)st->currentSpeed * st->rlRatio) * st->rlDir;
    int rr = (int)((float)st->currentSpeed * st->rrRatio) * st->rrDir;

    evo_motor_run_speed_control_c(self->frontLeft,  (mp_float_t)fl);
    evo_motor_run_speed_control_c(self->frontRight, (mp_float_t)fr);
    evo_motor_run_speed_control_c(self->rearLeft,   (mp_float_t)rl);
    evo_motor_run_speed_control_c(self->rearRight,  (mp_float_t)rr);
}

static void mecanum_apply_sync_speed(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    uint32_t now = mp_hal_ticks_ms();
    float dt_s = (st->syncLastMs == 0)
        ? ((float)EVO_MECANUM_LOOP_MS / 1000.0f)
        : ((float)(now - st->syncLastMs) / 1000.0f);
    if (dt_s <= 0.0f) {
        dt_s = (float)EVO_MECANUM_LOOP_MS / 1000.0f;
    }
    st->syncLastMs = now;

    int flAngle, frAngle, rlAngle, rrAngle;
    mecanum_get_abs_angles(self, st, &flAngle, &frAngle, &rlAngle, &rrAngle);

    float flProg = mecanum_virtual_progress_for_wheel(flAngle, st->flRatio);
    float frProg = mecanum_virtual_progress_for_wheel(frAngle, st->frRatio);
    float rlProg = mecanum_virtual_progress_for_wheel(rlAngle, st->rlRatio);
    float rrProg = mecanum_virtual_progress_for_wheel(rrAngle, st->rrRatio);

    float sum = 0.0f;
    int count = 0;
    if (st->flRatio > 0.0f) { sum += flProg; count++; }
    if (st->frRatio > 0.0f) { sum += frProg; count++; }
    if (st->rlRatio > 0.0f) { sum += rlProg; count++; }
    if (st->rrRatio > 0.0f) { sum += rrProg; count++; }
    float refProg = (count == 0) ? 0.0f : (sum / (float)count);

    int flBase = (int)((float)st->currentSpeed * st->flRatio);
    int frBase = (int)((float)st->currentSpeed * st->frRatio);
    int rlBase = (int)((float)st->currentSpeed * st->rlRatio);
    int rrBase = (int)((float)st->currentSpeed * st->rrRatio);

    int correction_limit = MAX(1, (int)((float)MAX(1, st->currentSpeed) * EVO_MECANUM_SYNC_CORRECTION_LIMIT_RATIO));

    int fl = mecanum_apply_pid_to_wheel(self, flBase, st->flDir, st->flRatio,
                                        refProg, flProg, &st->flErrPrev, &st->flErrInt,
                                        dt_s, correction_limit, st->maxSpeed);
    int fr = mecanum_apply_pid_to_wheel(self, frBase, st->frDir, st->frRatio,
                                        refProg, frProg, &st->frErrPrev, &st->frErrInt,
                                        dt_s, correction_limit, st->maxSpeed);
    int rl = mecanum_apply_pid_to_wheel(self, rlBase, st->rlDir, st->rlRatio,
                                        refProg, rlProg, &st->rlErrPrev, &st->rlErrInt,
                                        dt_s, correction_limit, st->maxSpeed);
    int rr = mecanum_apply_pid_to_wheel(self, rrBase, st->rrDir, st->rrRatio,
                                        refProg, rrProg, &st->rrErrPrev, &st->rrErrInt,
                                        dt_s, correction_limit, st->maxSpeed);

    evo_motor_run_speed_control_c(self->frontLeft,  (mp_float_t)fl);
    evo_motor_run_speed_control_c(self->frontRight, (mp_float_t)fr);
    evo_motor_run_speed_control_c(self->rearLeft,   (mp_float_t)rl);
    evo_motor_run_speed_control_c(self->rearRight,  (mp_float_t)rr);
}

static void mecanum_init_move_degrees(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    mecanum_prepare_common(self, st);
    st->degrees = abs_i(st->degrees);

    st->flStart = (int)evo_motor_get_angle_deg(self->frontLeft);
    st->frStart = (int)evo_motor_get_angle_deg(self->frontRight);
    st->rlStart = (int)evo_motor_get_angle_deg(self->rearLeft);
    st->rrStart = (int)evo_motor_get_angle_deg(self->rearRight);
    st->syncLastMs = 0;

    if (st->accel > 0 && st->maxSpeed > st->startSpeed) {
        st->accelDist = ((st->maxSpeed * st->maxSpeed) - (st->startSpeed * st->startSpeed)) / (2 * st->accel);
    } else {
        st->accelDist = 0;
    }

    if (st->decel > 0 && st->maxSpeed > st->endSpeed) {
        st->decelDist = ((st->maxSpeed * st->maxSpeed) - (st->endSpeed * st->endSpeed)) / (2 * st->decel);
    } else {
        st->decelDist = 0;
    }

    if (st->accelDist + st->decelDist > st->degrees && st->accelDist + st->decelDist > 0) {
        float scale = (float)st->degrees / (float)(st->accelDist + st->decelDist);
        st->accelDist = (int)((float)st->accelDist * scale);
        st->decelDist = st->degrees - st->accelDist;
    }
}

static bool mecanum_step_move_degrees(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    int progress = mecanum_angle_progress(self, st);
    if (st->degrees <= 0 || st->maxSpeed <= 0 || mecanum_degrees_complete(self, st)) {
        mecanum_apply_stop_now(self, st->stopBehavior);
        return true;
    }

    st->currentSpeed = mecanum_calc_degrees_profile_speed(self, st, progress);
    mecanum_apply_sync_speed(self, st);
    return false;
}

static void mecanum_run_move_degrees(evo_mecanum_obj_t *self,
                                     int xSpeed,
                                     int ySpeed,
                                     int turnSpeed,
                                     int degrees,
                                     int stopBehavior) {
    evo_mecanum_exec_t st;
    memset(&st, 0, sizeof(st));
    st.xSpeed = xSpeed;
    st.ySpeed = ySpeed;
    st.turnSpeed = turnSpeed;
    st.degrees = degrees;
    st.stopBehavior = stopBehavior;

    self->busy = true;
    mecanum_init_move_degrees(self, &st);

    while (1) {
        MICROPY_EVENT_POLL_HOOK;
        if (mecanum_step_move_degrees(self, &st)) {
            break;
        }
        mp_hal_delay_ms(EVO_MECANUM_LOOP_MS);
    }

    self->busy = false;
}

static void mecanum_init_move_time(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    mecanum_prepare_common(self, st);
    st->timems = MAX(0, st->timems);
    st->slowdowntime = MAX(0, st->slowdowntime);
    st->t0_ms = mp_hal_ticks_ms();

    st->flStart = (int)evo_motor_get_angle_deg(self->frontLeft);
    st->frStart = (int)evo_motor_get_angle_deg(self->frontRight);
    st->rlStart = (int)evo_motor_get_angle_deg(self->rearLeft);
    st->rrStart = (int)evo_motor_get_angle_deg(self->rearRight);
    st->syncLastMs = 0;

    if (st->accel > 0 && st->maxSpeed > st->startSpeed) {
        st->accelTimeMs = ((st->maxSpeed - st->startSpeed) * 1000) / st->accel;
    } else {
        st->accelTimeMs = 0;
    }
}

static bool mecanum_step_move_time(evo_mecanum_obj_t *self, evo_mecanum_exec_t *st) {
    uint32_t elapsed = mp_hal_ticks_ms() - st->t0_ms;
    if (elapsed >= (uint32_t)st->timems || st->timems <= 0 || st->maxSpeed <= 0) {
        mecanum_apply_stop_now(self, st->stopBehavior);
        return true;
    }

    st->currentSpeed = mecanum_calc_time_profile_speed(self, st, elapsed);
    mecanum_apply_sync_speed(self, st);
    return false;
}

static void mecanum_run_move_time(evo_mecanum_obj_t *self,
                                  int xSpeed,
                                  int ySpeed,
                                  int turnSpeed,
                                  int timems,
                                  int slowdowntime,
                                  int stopBehavior) {
    evo_mecanum_exec_t st;
    memset(&st, 0, sizeof(st));
    st.xSpeed = xSpeed;
    st.ySpeed = ySpeed;
    st.turnSpeed = turnSpeed;
    st.timems = timems;
    st.slowdowntime = slowdowntime;
    st.stopBehavior = stopBehavior;

    self->busy = true;
    mecanum_init_move_time(self, &st);

    while (1) {
        MICROPY_EVENT_POLL_HOOK;
        if (mecanum_step_move_time(self, &st)) {
            break;
        }
        mp_hal_delay_ms(EVO_MECANUM_LOOP_MS);
    }

    self->busy = false;
}

static mp_obj_t evo_mecanum_make_new(const mp_obj_type_t *type,
                                     size_t n_args,
                                     size_t n_kw,
                                     const mp_obj_t *all_args) {
    enum { ARG_frontLeft, ARG_frontRight, ARG_rearLeft, ARG_rearRight };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_frontLeft,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_frontRight, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_rearLeft,   MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_rearRight,  MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    evo_mecanum_obj_t *self = mp_obj_malloc(evo_mecanum_obj_t, type);

    self->frontLeft  = (evo_motor_obj_t *)MP_OBJ_TO_PTR(args[ARG_frontLeft].u_obj);
    self->frontRight = (evo_motor_obj_t *)MP_OBJ_TO_PTR(args[ARG_frontRight].u_obj);
    self->rearLeft   = (evo_motor_obj_t *)MP_OBJ_TO_PTR(args[ARG_rearLeft].u_obj);
    self->rearRight  = (evo_motor_obj_t *)MP_OBJ_TO_PTR(args[ARG_rearRight].u_obj);

    self->startSpeed = 800;
    self->endSpeed = 800;
    self->accel = 10000;
    self->decel = 10000;
    self->accelerationProfile = EVO_ACCEL_TRAPEZOID;
    self->stopBehavior = EVO_STOP_BRAKE;
    self->kpSync = EVO_MECANUM_SYNC_KP_DEFAULT;
    self->kiSync = EVO_MECANUM_SYNC_KI_DEFAULT;
    self->kdSync = EVO_MECANUM_SYNC_KD_DEFAULT;
    self->busy = false;

    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t mp_setStartSpeed(mp_obj_t self_in, mp_obj_t v_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->startSpeed = abs_i(obj_get_rounded_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setStartSpeed_obj, mp_setStartSpeed);

static mp_obj_t mp_setEndSpeed(mp_obj_t self_in, mp_obj_t v_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->endSpeed = abs_i(obj_get_rounded_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setEndSpeed_obj, mp_setEndSpeed);

static mp_obj_t mp_setAcceleration(mp_obj_t self_in, mp_obj_t v_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->accel = abs_i(mp_obj_get_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setAcceleration_obj, mp_setAcceleration);

static mp_obj_t mp_getAcceleration(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->accel);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getAcceleration_obj, mp_getAcceleration);

static mp_obj_t mp_setDeceleration(mp_obj_t self_in, mp_obj_t v_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->decel = abs_i(mp_obj_get_int(v_in));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setDeceleration_obj, mp_setDeceleration);

static mp_obj_t mp_getDeceleration(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->decel);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getDeceleration_obj, mp_getDeceleration);

static mp_obj_t mp_setAccelerationProfile(mp_obj_t self_in, mp_obj_t profile_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->accelerationProfile = normalize_acceleration_profile(profile_in);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setAccelerationProfile_obj, mp_setAccelerationProfile);

static mp_obj_t mp_getAccelerationProfile(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_int(self->accelerationProfile);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getAccelerationProfile_obj, mp_getAccelerationProfile);

static mp_obj_t mp_setStopBehavior(mp_obj_t self_in, mp_obj_t stop_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->stopBehavior = normalize_stop_behavior_obj(stop_in);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setStopBehavior_obj, mp_setStopBehavior);
static MP_DEFINE_CONST_FUN_OBJ_2(mp_setStopBehaviour_obj, mp_setStopBehavior);

static mp_obj_t mp_setSyncPID(size_t n_args, const mp_obj_t *args) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    (void)n_args;

    self->kpSync = mp_obj_get_float(args[1]);
    self->kiSync = mp_obj_get_float(args[2]);
    self->kdSync = mp_obj_get_float(args[3]);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_setSyncPID_obj, 4, 4, mp_setSyncPID);

static mp_obj_t mp_getSyncPID(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);

    mp_obj_t tuple[3] = {
        mp_obj_new_float(self->kpSync),
        mp_obj_new_float(self->kiSync),
        mp_obj_new_float(self->kdSync),
    };

    return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_getSyncPID_obj, mp_getSyncPID);

static mp_obj_t mp_movePower(size_t n_args, const mp_obj_t *args) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    (void)n_args;
    int xSpeed = obj_get_rounded_int(args[1]);
    int ySpeed = obj_get_rounded_int(args[2]);
    int turnSpeed = obj_get_rounded_int(args[3]);

    evo_mecanum_exec_t st;
    memset(&st, 0, sizeof(st));
    st.xSpeed = xSpeed;
    st.ySpeed = ySpeed;
    st.turnSpeed = turnSpeed;
    mecanum_calc_wheel_targets(&st);

    evo_motor_run_power_c(self->frontLeft,  st.flTarget);
    evo_motor_run_power_c(self->frontRight, st.frTarget);
    evo_motor_run_power_c(self->rearLeft,   st.rlTarget);
    evo_motor_run_power_c(self->rearRight,  st.rrTarget);
    self->busy = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_movePower_obj, 4, 4, mp_movePower);

static mp_obj_t mp_moveSpeed(size_t n_args, const mp_obj_t *args) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    (void)n_args;
    int xSpeed = obj_get_rounded_int(args[1]);
    int ySpeed = obj_get_rounded_int(args[2]);
    int turnSpeed = obj_get_rounded_int(args[3]);

    evo_mecanum_exec_t st;
    memset(&st, 0, sizeof(st));
    st.xSpeed = xSpeed;
    st.ySpeed = ySpeed;
    st.turnSpeed = turnSpeed;
    mecanum_prepare_common(self, &st);
    st.currentSpeed = st.maxSpeed;
    mecanum_apply_speed(self, &st);
    self->busy = false;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_moveSpeed_obj, 4, 4, mp_moveSpeed);
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_move_obj, 4, 4, mp_moveSpeed);

static mp_obj_t mp_moveDegrees(size_t n_args, const mp_obj_t *args) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int xSpeed = obj_get_rounded_int(args[1]);
    int ySpeed = obj_get_rounded_int(args[2]);
    int turnSpeed = obj_get_rounded_int(args[3]);
    int degrees = mp_obj_get_int(args[4]);
    int stopBehavior = (n_args >= 6)
        ? normalize_stop_behavior_obj(args[5])
        : self->stopBehavior;

    mecanum_run_move_degrees(self, xSpeed, ySpeed, turnSpeed, degrees, stopBehavior);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_moveDegrees_obj, 5, 6, mp_moveDegrees);

static mp_obj_t mp_moveTime(size_t n_args, const mp_obj_t *args) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int xSpeed = obj_get_rounded_int(args[1]);
    int ySpeed = obj_get_rounded_int(args[2]);
    int turnSpeed = obj_get_rounded_int(args[3]);
    int timems = mp_obj_get_int(args[4]);
    int slowdowntime = (n_args >= 6) ? mp_obj_get_int(args[5]) : 200;
    int stopBehavior = (n_args >= 7)
        ? normalize_stop_behavior_obj(args[6])
        : self->stopBehavior;

    if (timems < 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("time must be >= 0"));
    }

    mecanum_run_move_time(self, xSpeed, ySpeed, turnSpeed, timems, slowdowntime, stopBehavior);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_moveTime_obj, 5, 7, mp_moveTime);

static mp_obj_t mp_hold(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    mecanum_apply_stop_now(self, EVO_STOP_HOLD);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_hold_obj, mp_hold);

static mp_obj_t mp_brake(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    mecanum_apply_stop_now(self, EVO_STOP_BRAKE);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_brake_obj, mp_brake);

static mp_obj_t mp_coast(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    mecanum_apply_stop_now(self, EVO_STOP_COAST);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_coast_obj, mp_coast);

static mp_obj_t mp_stop(size_t n_args, const mp_obj_t *args) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int stopBehavior = (n_args >= 2)
        ? normalize_stop_behavior_obj(args[1])
        : EVO_STOP_BRAKE;
    self->busy = false;
    mecanum_apply_stop_now(self, stopBehavior);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_stop_obj, 1, 2, mp_stop);

static mp_obj_t mp_resetAngle(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    evo_motor_reset_angle(self->frontLeft);
    evo_motor_reset_angle(self->frontRight);
    evo_motor_reset_angle(self->rearLeft);
    evo_motor_reset_angle(self->rearRight);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_resetAngle_obj, mp_resetAngle);

static mp_obj_t mp_isBusy(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->busy);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mp_isBusy_obj, mp_isBusy);

static mp_obj_t evo_mecanum_deinit_method(mp_obj_t self_in) {
    evo_mecanum_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->busy = false;
    mecanum_apply_stop_now(self, EVO_STOP_BRAKE);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(evo_mecanum_deinit_method_obj, evo_mecanum_deinit_method);

static const mp_rom_map_elem_t evo_mecanum_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_setStartSpeed),          MP_ROM_PTR(&mp_setStartSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_setEndSpeed),            MP_ROM_PTR(&mp_setEndSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_setAcceleration),        MP_ROM_PTR(&mp_setAcceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_getAcceleration),        MP_ROM_PTR(&mp_getAcceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_setDeceleration),        MP_ROM_PTR(&mp_setDeceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_getDeceleration),        MP_ROM_PTR(&mp_getDeceleration_obj) },
    { MP_ROM_QSTR(MP_QSTR_setAccelerationProfile), MP_ROM_PTR(&mp_setAccelerationProfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_getAccelerationProfile), MP_ROM_PTR(&mp_getAccelerationProfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_setStopBehavior),        MP_ROM_PTR(&mp_setStopBehavior_obj) },
    { MP_ROM_QSTR(MP_QSTR_setStopBehaviour),       MP_ROM_PTR(&mp_setStopBehaviour_obj) },
    { MP_ROM_QSTR(MP_QSTR_setSyncPID),             MP_ROM_PTR(&mp_setSyncPID_obj) },
    { MP_ROM_QSTR(MP_QSTR_getSyncPID),             MP_ROM_PTR(&mp_getSyncPID_obj) },

    { MP_ROM_QSTR(MP_QSTR_move),                   MP_ROM_PTR(&mp_move_obj) },
    { MP_ROM_QSTR(MP_QSTR_moveSpeed),              MP_ROM_PTR(&mp_moveSpeed_obj) },
    { MP_ROM_QSTR(MP_QSTR_movePower),              MP_ROM_PTR(&mp_movePower_obj) },
    { MP_ROM_QSTR(MP_QSTR_moveDegrees),            MP_ROM_PTR(&mp_moveDegrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_runDegrees),             MP_ROM_PTR(&mp_moveDegrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_moveTime),               MP_ROM_PTR(&mp_moveTime_obj) },
    { MP_ROM_QSTR(MP_QSTR_runTime),                MP_ROM_PTR(&mp_moveTime_obj) },

    { MP_ROM_QSTR(MP_QSTR_hold),                   MP_ROM_PTR(&mp_hold_obj) },
    { MP_ROM_QSTR(MP_QSTR_brake),                  MP_ROM_PTR(&mp_brake_obj) },
    { MP_ROM_QSTR(MP_QSTR_coast),                  MP_ROM_PTR(&mp_coast_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),                   MP_ROM_PTR(&mp_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_resetAngle),             MP_ROM_PTR(&mp_resetAngle_obj) },
    { MP_ROM_QSTR(MP_QSTR_isBusy),                 MP_ROM_PTR(&mp_isBusy_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),                 MP_ROM_PTR(&evo_mecanum_deinit_method_obj) },
};
static MP_DEFINE_CONST_DICT(evo_mecanum_locals_dict, evo_mecanum_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    evo_mecanum_type,
    MP_QSTR_EvoMecanum,
    MP_TYPE_FLAG_NONE,
    make_new, evo_mecanum_make_new,
    locals_dict, &evo_mecanum_locals_dict
);
