"""Host regression test: python3 tools/tests/test_motorpair_stall.py (requires cc)."""
from pathlib import Path
import subprocess
import tempfile


SOURCE = Path(__file__).resolve().parents[2] / "ports/esp32/usermod/evo_motorpair.c"


def function(source, name):
    start = source.index("static ", source.rfind("\n", 0, source.index(name + "(")))
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    source = SOURCE.read_text()
    state_start = source.index("typedef struct {")
    state_end = source.index("} evo_pair_exec_t;", state_start) + len("} evo_pair_exec_t;")
    harness = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define EVO_PAIR_INTEGRAL_BUF_SIZE 20
typedef struct { int32_t position; } motor_t;
typedef struct { motor_t *m1, *m2; uint32_t stallTimeoutMs; } evo_motorpair_obj_t;
static uint32_t now;
static int stops, stopped_with, commands;
static uint32_t mp_hal_ticks_ms(void) { return now; }
""" + source[state_start:state_end] + r"""
static void pair_apply_stop_now(evo_motorpair_obj_t *s, int mode) {
    (void)s; stops++; stopped_with = mode;
}
static void pair_update_encoder_state(evo_motorpair_obj_t *s, evo_pair_exec_t *st) {
    (void)s; (void)st;
}
static int pair_calc_degrees_profile_speed(evo_motorpair_obj_t *s, evo_pair_exec_t *st, int p) {
    (void)s; (void)st; (void)p; return 100;
}
static int pair_calc_time_profile_speed(evo_motorpair_obj_t *s, evo_pair_exec_t *st, int p) {
    (void)s; (void)st; (void)p; return 100;
}
static void pair_apply_speed_and_sync(evo_motorpair_obj_t *s, evo_pair_exec_t *st) {
    (void)s; (void)st; commands++;
}
"""
    for name in ("pair_is_stalled", "pair_step_move_degrees", "pair_step_move_time"):
        harness += function(source, name) + "\n"
    harness += r"""
int main(void) {
    motor_t left = {0}, right = {0};
    evo_motorpair_obj_t pair = {&left, &right, 1000};
    evo_pair_exec_t st = {.leftSpeed = 100, .rightSpeed = 100};
    now = 999; assert(!pair_is_stalled(&pair, &st));
    now = 1000; assert(pair_is_stalled(&pair, &st));

    // A single raw count (including reverse motion) refreshes each timer.
    left.position = 1; right.position = -1;
    assert(!pair_is_stalled(&pair, &st));
    now = 1999; assert(!pair_is_stalled(&pair, &st));
    now = 2000; right.position--;
    assert(pair_is_stalled(&pair, &st)); // moving right cannot hide stalled left
    left.position++;
    assert(!pair_is_stalled(&pair, &st));
    now = 3000; left.position++;
    assert(pair_is_stalled(&pair, &st)); // moving left cannot hide stalled right

    // Pivot wheels commanded to zero are excluded; either pivot direction works.
    st.rightSpeed = 0;
    assert(!pair_is_stalled(&pair, &st));
    now = 4000; left.position++;
    assert(!pair_is_stalled(&pair, &st));
    st.leftSpeed = 0; st.rightSpeed = -100; right.position--;
    assert(!pair_is_stalled(&pair, &st));
    now = 5000; assert(pair_is_stalled(&pair, &st));
    st.rightSpeed = 0; assert(!pair_is_stalled(&pair, &st));

    st.leftSpeed = 100; pair.stallTimeoutMs = 0;
    now = 100000; assert(!pair_is_stalled(&pair, &st));
    pair.stallTimeoutMs = 50;
    st.lastLeftUpdateMs = UINT32_MAX - 20;
    now = 28; assert(!pair_is_stalled(&pair, &st));
    now = 29; assert(pair_is_stalled(&pair, &st));

    // Real movement steps stop once, preserve the requested mode, and issue no
    // further motor commands when stalled, for degree and timed movements.
    for (int timed = 0; timed < 2; timed++) {
        for (int mode = 0; mode < 3; mode++) {
            st = (evo_pair_exec_t){.leftSpeed = 100, .rightSpeed = 100,
                .degrees = 10000, .timems = 10000, .stopBehavior = mode,
                .lastLeftPosition = left.position, .lastRightPosition = right.position};
            pair.stallTimeoutMs = 1000;
            stops = commands = 0;
            now = 999;
            assert(!(timed ? pair_step_move_time(&pair, &st)
                           : pair_step_move_degrees(&pair, &st)));
            assert(commands == 1 && stops == 0);
            now = 1000;
            assert(timed ? pair_step_move_time(&pair, &st)
                         : pair_step_move_degrees(&pair, &st));
            assert(commands == 1 && stops == 1 && stopped_with == mode);
        }
    }
    puts("motorpair stall tests passed");
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
