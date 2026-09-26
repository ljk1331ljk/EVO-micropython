"""Host regression test: python3 tools/tests/test_motor_stall.py (requires cc)."""
from pathlib import Path
import subprocess
import tempfile

from test_motorpair_stall import function


def main():
    source = (Path(__file__).resolve().parents[2] /
              "ports/esp32/usermod/evo_motor.c").read_text()
    harness = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
typedef float mp_float_t;
typedef intptr_t mp_obj_t;
typedef struct {
    int32_t position, stall_last_position;
    uint32_t stall_timeout_ms, stall_last_update_ms;
    bool stalled;
    int cpr;
} evo_motor_obj_t;
#define MP_OBJ_TO_PTR(o) ((void *)(o))
#define mp_const_none 0
#define MP_ERROR_TEXT(s) s
#define MICROPY_EVENT_POLL_HOOK ((void)0)
static int mp_obj_get_int(mp_obj_t o) { return (int)o; }
static float mp_obj_get_float(mp_obj_t o) { return (float)o; }
static mp_obj_t mp_obj_new_bool(bool b) { return b; }
static void mp_raise_ValueError(const char *s) { (void)s; abort(); }
static uint32_t now;
static int stops, commands, power;
static bool encoder_moving;
static evo_motor_obj_t *active;
static uint32_t mp_hal_ticks_ms(void) { return now; }
static void mp_hal_delay_ms(int ms) {
    now += ms;
    if (encoder_moving) active->position += power > 0 ? 1 : -1;
    assert(now < 100000); // catches a movement that never exits
}
static void evo_motor_cancel_hold(evo_motor_obj_t *m) { (void)m; }
static void evo_motor_reset_speed_state(evo_motor_obj_t *m) { (void)m; }
static void evo_motor_run_power_c(evo_motor_obj_t *m, int p) {
    (void)m; power = p; commands++;
}
static void evo_motor_run_speed_control_c(evo_motor_obj_t *m, float p) {
    (void)m; power = (int)p;
    if (p == 0) stops++; else commands++;
}
"""
    for name in ("motor_clear_stall", "motor_check_stall", "evo_motor_clearStall",
                 "evo_motor_getStalled", "evo_motor_runPower", "evo_motor_run",
                 "evo_motor_runSpeed", "evo_motor_runTime", "evo_motor_runAngle"):
        harness += function(source, name) + "\n"
    harness += r"""
int main(void) {
    evo_motor_obj_t m = {.stall_timeout_ms = 1000, .cpr = 360};
    active = &m;
    mp_obj_t obj = (mp_obj_t)&m;
    evo_motor_clearStall(obj);
    evo_motor_run(obj, 100);
    now = 999; evo_motor_runPower(obj, 200);
    assert(!evo_motor_getStalled(obj));
    now = 1000; evo_motor_runSpeed(obj, 100);
    assert(evo_motor_getStalled(obj) && stops == 0 && commands == 3);
    m.position++;
    evo_motor_run(obj, 100);
    assert(evo_motor_getStalled(obj));
    evo_motor_clearStall(obj);
    assert(!evo_motor_getStalled(obj) && m.stall_last_update_ms == now);
    assert(m.stall_last_position == m.position);
    now = 1999; evo_motor_runSpeed(obj, -100);
    assert(!m.stalled);
    now = 2000; evo_motor_runSpeed(obj, -100);
    assert(m.stalled);
    evo_motor_clearStall(obj);
    now = 4000; evo_motor_run(obj, 0);
    assert(!m.stalled);
    m.stall_timeout_ms = 0;
    now = 10000; evo_motor_run(obj, 100);
    assert(!m.stalled);
    m.stall_timeout_ms = 50;
    now = UINT32_MAX - 20; evo_motor_clearStall(obj);
    now = 28; evo_motor_run(obj, 100); assert(!m.stalled);
    now = 29; evo_motor_run(obj, 100); assert(m.stalled);

    // Execute the real blocking methods with frozen and moving encoders.
    for (int direction = -1; direction <= 1; direction += 2) {
        now = 0; stops = 0; encoder_moving = false;
        evo_motor_runAngle(obj, 100, direction * 360);
        assert(m.stalled && stops == 1 && now == 50);
        now = 0; stops = 0; encoder_moving = true;
        evo_motor_runAngle(obj, 100, direction * 10);
        assert(!m.stalled && stops == 1 && now == 100);
    }
    now = 0; stops = 0; encoder_moving = false;
    evo_motor_runTime(obj, 100, 10);
    assert(m.stalled && stops == 1 && now == 50);
    now = 0; stops = 0; encoder_moving = true;
    evo_motor_runTime(obj, 100, 1);
    assert(!m.stalled && stops == 1 && now == 1000);
    m.stalled = true;
    evo_motor_runAngle(obj, 100, 0);
    assert(!m.stalled);
    m.stalled = true;
    evo_motor_runTime(obj, 100, 0);
    assert(!m.stalled);
    puts("individual motor stall tests passed");
}
"""
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp)
        (path / "stall.c").write_text(harness)
        subprocess.run(["cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        str(path / "stall.c"), "-o", str(path / "stall")], check=True)
        subprocess.run([str(path / "stall")], check=True)


if __name__ == "__main__":
    main()
