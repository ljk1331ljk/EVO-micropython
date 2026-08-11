#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "py/mphal.h"
#include "py/runtime.h"

#include "evo_linetrace.h"
#include "evo_motor.h"
#include "evo_motorpair.h"

#define EVO_LINE_LOOP_MS (1)

typedef struct _evo_linetrace_obj_t {
    mp_obj_base_t base;
    evo_motorpair_obj_t *robot;
    mp_obj_t left_sensor;
    mp_obj_t right_sensor;
    mp_float_t kp;
    mp_float_t ki;
    mp_float_t kd;
    int left_min;
    int left_max;
    int right_min;
    int right_max;
} evo_linetrace_obj_t;

typedef struct _evo_line_pid_t {
    mp_float_t integral;
    mp_float_t previous_error;
    uint32_t previous_ms;
    bool initialized;
} evo_line_pid_t;

static inline int clamp_i(int value, int minimum, int maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static inline int abs_i(int value) {
    return value < 0 ? -value : value;
}

static int read_raw_clear(mp_obj_t sensor) {
    mp_obj_t destination[2];
    mp_load_method(sensor, MP_QSTR_getRawClear, destination);
    return mp_obj_get_int(mp_call_method_n_kw(0, 0, destination));
}

static int map_reading(int raw, int input_min, int input_max) {
    if (input_min == -1 && input_max == -1) {
        return raw;
    }
    int64_t scaled = ((int64_t)(raw - input_min) * 500) / (input_max - input_min);
    return clamp_i((int)scaled, 0, 500);
}

static void read_calibrated(evo_linetrace_obj_t *self, int *left, int *right) {
    *left = map_reading(read_raw_clear(self->left_sensor), self->left_min, self->left_max);
    *right = map_reading(read_raw_clear(self->right_sensor), self->right_min, self->right_max);
}

static bool junction_reached(int left, int right, int type, int threshold) {
    bool left_crossed = left <= threshold;
    bool right_crossed = right <= threshold;
    if (type == EVO_JUNCTION_LEFT) return left_crossed;
    if (type == EVO_JUNCTION_RIGHT) return right_crossed;
    return left_crossed && right_crossed;
}

static void validate_junction_type(int type) {
    if (type < EVO_JUNCTION_LEFT || type > EVO_JUNCTION_BOTH) {
        mp_raise_ValueError(MP_ERROR_TEXT("junction type must be LEFT, RIGHT or BOTH"));
    }
}

static void validate_tracking_side(int side) {
    if (side != EVO_JUNCTION_LEFT && side != EVO_JUNCTION_RIGHT) {
        mp_raise_ValueError(MP_ERROR_TEXT("side must be LEFT or RIGHT"));
    }
}

static void finish_move(evo_motorpair_obj_t *robot, bool stop) {
    robot->busy = false;
    if (stop) {
        evo_motor_brake_c(robot->m1);
        evo_motor_brake_c(robot->m2);
    } else {
        evo_motor_coast_c(robot->m1);
        evo_motor_coast_c(robot->m2);
    }
}

static int pid_correction(evo_linetrace_obj_t *self, evo_line_pid_t *pid, mp_float_t error) {
    uint32_t now = mp_hal_ticks_ms();
    mp_float_t derivative = 0;
    if (pid->initialized) {
        uint32_t elapsed_ms = now - pid->previous_ms;
        if (elapsed_ms > 0) {
            mp_float_t dt = (mp_float_t)elapsed_ms / 1000.0f;
            pid->integral += error * dt;
            derivative = (error - pid->previous_error) / dt;
        }
    } else {
        pid->initialized = true;
    }
    pid->previous_error = error;
    pid->previous_ms = now;
    return (int)roundf(self->kp * error + self->ki * pid->integral + self->kd * derivative);
}

static void drive_pid(evo_linetrace_obj_t *self, int speed, mp_float_t error, evo_line_pid_t *pid) {
    int correction = pid_correction(self, pid, error);
    int direction = speed < 0 ? -1 : 1;
    int limit = EVO_PWM_MAX;
    int left = clamp_i(speed + direction * correction, -limit, limit);
    int right = clamp_i(speed - direction * correction, -limit, limit);
    self->robot->busy = true;
    evo_motor_run_power_c(self->robot->m1, left);
    evo_motor_run_power_c(self->robot->m2, right);
}

static int degree_progress(evo_motorpair_obj_t *robot, int left_start, int right_start) {
    int left = abs_i((int)evo_motor_get_angle_deg(robot->m1) - left_start);
    int right = abs_i((int)evo_motor_get_angle_deg(robot->m2) - right_start);
    return (left + right) / 2;
}

static mp_obj_t evo_linetrace_make_new(const mp_obj_type_t *type, size_t n_args,
                                       size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 3, 3, false);
    if (!mp_obj_is_type(args[0], &evo_motorpair_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT("robot must be EvoMotorPair"));
    }

    // Validate the sensor interface at construction time.
    mp_load_attr(args[1], MP_QSTR_getRawClear);
    mp_load_attr(args[2], MP_QSTR_getRawClear);

    evo_linetrace_obj_t *self = mp_obj_malloc(evo_linetrace_obj_t, type);
    self->robot = MP_OBJ_TO_PTR(args[0]);
    self->left_sensor = args[1];
    self->right_sensor = args[2];
    self->kp = 1.0f;
    self->ki = 0.0f;
    self->kd = 0.0f;
    self->left_min = -1;
    self->left_max = -1;
    self->right_min = -1;
    self->right_max = -1;
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t set_pid(size_t n_args, const mp_obj_t *args) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    self->kp = mp_obj_get_float(args[1]);
    self->ki = mp_obj_get_float(args[2]);
    self->kd = mp_obj_get_float(args[3]);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(set_pid_obj, 4, 4, set_pid);

static mp_obj_t get_pid(mp_obj_t self_in) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_obj_t values[] = {
        mp_obj_new_float(self->kp), mp_obj_new_float(self->ki), mp_obj_new_float(self->kd),
    };
    return mp_obj_new_tuple(3, values);
}
static MP_DEFINE_CONST_FUN_OBJ_1(get_pid_obj, get_pid);

static mp_obj_t calibrate(size_t n_args, const mp_obj_t *args) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int left_min = mp_obj_get_int(args[1]);
    int left_max = mp_obj_get_int(args[2]);
    int right_min = mp_obj_get_int(args[3]);
    int right_max = mp_obj_get_int(args[4]);
    bool left_disabled = left_min == -1 && left_max == -1;
    bool right_disabled = right_min == -1 && right_max == -1;
    if ((!left_disabled && left_min >= left_max) || (!right_disabled && right_min >= right_max) ||
        ((left_min == -1) != (left_max == -1)) || ((right_min == -1) != (right_max == -1))) {
        mp_raise_ValueError(MP_ERROR_TEXT("use -1, -1 or a minimum less than maximum"));
    }
    self->left_min = left_min;
    self->left_max = left_max;
    self->right_min = right_min;
    self->right_max = right_max;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calibrate_obj, 5, 5, calibrate);

static mp_obj_t read_calibrated_readings(mp_obj_t self_in) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int left, right;
    read_calibrated(self, &left, &right);
    mp_obj_t values[] = {mp_obj_new_int(left), mp_obj_new_int(right)};
    return mp_obj_new_tuple(2, values);
}
static MP_DEFINE_CONST_FUN_OBJ_1(read_calibrated_readings_obj, read_calibrated_readings);

static mp_obj_t double_degrees(size_t n_args, const mp_obj_t *args) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int speed = mp_obj_get_int(args[1]);
    int degrees = mp_obj_get_int(args[2]);
    bool stop = mp_obj_is_true(args[3]);
    if (degrees < 0) mp_raise_ValueError(MP_ERROR_TEXT("degrees must be >= 0"));
    int left_start = evo_motor_get_angle_deg(self->robot->m1);
    int right_start = evo_motor_get_angle_deg(self->robot->m2);
    evo_line_pid_t pid = {0};
    while (degree_progress(self->robot, left_start, right_start) < degrees) {
        int left, right;
        read_calibrated(self, &left, &right);
        drive_pid(self, speed, (mp_float_t)(left - right), &pid);
        MICROPY_EVENT_POLL_HOOK;
        mp_hal_delay_ms(EVO_LINE_LOOP_MS);
    }
    finish_move(self->robot, stop);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(double_degrees_obj, 4, 4, double_degrees);

static mp_obj_t single_degrees(size_t n_args, const mp_obj_t *args) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int side = mp_obj_get_int(args[1]);
    int speed = mp_obj_get_int(args[2]);
    int threshold = mp_obj_get_int(args[3]);
    int degrees = mp_obj_get_int(args[4]);
    bool stop = mp_obj_is_true(args[5]);
    validate_tracking_side(side);
    if (degrees < 0) mp_raise_ValueError(MP_ERROR_TEXT("degrees must be >= 0"));
    int left_start = evo_motor_get_angle_deg(self->robot->m1);
    int right_start = evo_motor_get_angle_deg(self->robot->m2);
    evo_line_pid_t pid = {0};
    while (degree_progress(self->robot, left_start, right_start) < degrees) {
        int left, right;
        read_calibrated(self, &left, &right);
        mp_float_t error = side == EVO_JUNCTION_LEFT
            ? (mp_float_t)(left - threshold)
            : (mp_float_t)(threshold - right);
        drive_pid(self, speed, error, &pid);
        MICROPY_EVENT_POLL_HOOK;
        mp_hal_delay_ms(EVO_LINE_LOOP_MS);
    }
    finish_move(self->robot, stop);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(single_degrees_obj, 6, 6, single_degrees);

static mp_obj_t double_junction(size_t n_args, const mp_obj_t *args) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int speed = mp_obj_get_int(args[1]);
    int junction_type = mp_obj_get_int(args[2]);
    int junction_threshold = mp_obj_get_int(args[3]);
    bool stop = mp_obj_is_true(args[4]);
    validate_junction_type(junction_type);
    evo_line_pid_t pid = {0};
    while (true) {
        int left, right;
        read_calibrated(self, &left, &right);
        if (junction_reached(left, right, junction_type, junction_threshold)) break;
        drive_pid(self, speed, (mp_float_t)(left - right), &pid);
        MICROPY_EVENT_POLL_HOOK;
        mp_hal_delay_ms(EVO_LINE_LOOP_MS);
    }
    finish_move(self->robot, stop);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(double_junction_obj, 5, 5, double_junction);

static mp_obj_t single_junction(size_t n_args, const mp_obj_t *args) {
    evo_linetrace_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    int side = mp_obj_get_int(args[1]);
    int speed = mp_obj_get_int(args[2]);
    int threshold = mp_obj_get_int(args[3]);
    int junction_type = mp_obj_get_int(args[4]);
    int junction_threshold = mp_obj_get_int(args[5]);
    bool stop = mp_obj_is_true(args[6]);
    validate_tracking_side(side);
    validate_junction_type(junction_type);
    evo_line_pid_t pid = {0};
    while (true) {
        int left, right;
        read_calibrated(self, &left, &right);
        if (junction_reached(left, right, junction_type, junction_threshold)) break;
        mp_float_t error = side == EVO_JUNCTION_LEFT
            ? (mp_float_t)(left - threshold)
            : (mp_float_t)(threshold - right);
        drive_pid(self, speed, error, &pid);
        MICROPY_EVENT_POLL_HOOK;
        mp_hal_delay_ms(EVO_LINE_LOOP_MS);
    }
    finish_move(self->robot, stop);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(single_junction_obj, 7, 7, single_junction);

static const mp_rom_map_elem_t evo_linetrace_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_LEFT), MP_ROM_INT(EVO_JUNCTION_LEFT) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT), MP_ROM_INT(EVO_JUNCTION_RIGHT) },
    { MP_ROM_QSTR(MP_QSTR_BOTH), MP_ROM_INT(EVO_JUNCTION_BOTH) },
    { MP_ROM_QSTR(MP_QSTR_setPIDParameters), MP_ROM_PTR(&set_pid_obj) },
    { MP_ROM_QSTR(MP_QSTR_getPIDParameters), MP_ROM_PTR(&get_pid_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrateColorSensor), MP_ROM_PTR(&calibrate_obj) },
    { MP_ROM_QSTR(MP_QSTR_readCalibratedReadings), MP_ROM_PTR(&read_calibrated_readings_obj) },
    { MP_ROM_QSTR(MP_QSTR_doubleLineFollowDegrees), MP_ROM_PTR(&double_degrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_doubleLineFollowJunction), MP_ROM_PTR(&double_junction_obj) },
    { MP_ROM_QSTR(MP_QSTR_singleLineFollowDegrees), MP_ROM_PTR(&single_degrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_singleLineFollowJunction), MP_ROM_PTR(&single_junction_obj) },
};
static MP_DEFINE_CONST_DICT(evo_linetrace_locals_dict, evo_linetrace_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    evo_linetrace_type,
    MP_QSTR_EvoLineTrace,
    MP_TYPE_FLAG_NONE,
    make_new, evo_linetrace_make_new,
    locals_dict, &evo_linetrace_locals_dict
);
