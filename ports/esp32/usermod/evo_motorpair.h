#ifndef EVO_MOTORPAIR_H
#define EVO_MOTORPAIR_H

#include <stdint.h>
#include <stdbool.h>

#include "py/obj.h"
#include "evo_motor.h"

#define EVO_ACCEL_NONE      (0)
#define EVO_ACCEL_TRAPEZOID (1)
#define EVO_ACCEL_SCURVE    (2)

#ifndef EVO_PAIR_INTEGRAL_BUF_SIZE
#define EVO_PAIR_INTEGRAL_BUF_SIZE 20
#endif

typedef struct _evo_motorpair_obj_t {
    mp_obj_base_t base;

    evo_motor_obj_t *m1;
    evo_motor_obj_t *m2;

    int startSpeed;
    int endSpeed;
    int accel;
    int decel;
    uint8_t accelerationProfile;

    int kpSync;
    int kiSync;
    int kdSync;

    uint8_t stopBehavior;
    bool busy;
} evo_motorpair_obj_t;

extern const mp_obj_type_t evo_motorpair_type;

#endif // EVO_MOTORPAIR_H
