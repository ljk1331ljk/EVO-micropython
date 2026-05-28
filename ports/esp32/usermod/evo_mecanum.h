#ifndef EVO_MECANUM_H
#define EVO_MECANUM_H

#include <stdint.h>
#include <stdbool.h>

#include "py/obj.h"
#include "evo_motor.h"

#ifndef EVO_ACCEL_NONE
#define EVO_ACCEL_NONE      (0)
#endif
#ifndef EVO_ACCEL_TRAPEZOID
#define EVO_ACCEL_TRAPEZOID (1)
#endif
#ifndef EVO_ACCEL_SCURVE
#define EVO_ACCEL_SCURVE    (2)
#endif

typedef struct _evo_mecanum_obj_t {
    mp_obj_base_t base;

    evo_motor_obj_t *frontLeft;
    evo_motor_obj_t *frontRight;
    evo_motor_obj_t *rearLeft;
    evo_motor_obj_t *rearRight;

    int startSpeed;
    int endSpeed;
    int accel;
    int decel;
    uint8_t accelerationProfile;
    uint8_t stopBehavior;

    float kpSync;
    float kiSync;
    float kdSync;

    bool busy;
} evo_mecanum_obj_t;

extern const mp_obj_type_t evo_mecanum_type;

#endif // EVO_MECANUM_H