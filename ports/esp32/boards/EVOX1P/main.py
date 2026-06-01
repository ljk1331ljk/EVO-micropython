import os
import time
from machine import Pin

import evo
from evo import *
from pins import *
from EvoOLED import *
from EvoBattery import *


# =========================
# Display and battery setup
# =========================

EvoDisplay = EvoOLED(I2CA, 7)
EvoDisplay.begin()

evo_battery = EvoBattery()


# =========================
# Button setup
# =========================

left_btn = Pin(BUTTON_L_PIN, Pin.IN, Pin.PULL_UP)
right_btn = Pin(BUTTON_R_PIN, Pin.IN, Pin.PULL_UP)
select_btn = Pin(BUTTON_C_PIN, Pin.IN, Pin.PULL_UP)


# =========================
# Project settings
# =========================

PROJECT_ROOT = "/programs"


# =========================
# Helper functions
# =========================

def show_line(text, line, clear=False, refresh=True):
    EvoDisplay.writeLineToDisplay(str(text), line, clear, refresh)


def get_header():
    try:
        name = evo.get_name()
    except:
        name = "Evo"

    try:
        voltage = evo_battery.getBatteryVoltage()
        return "{}     {:.1f}V".format(name, voltage)
    except:
        return "{} Battery?".format(name)


def button_pressed(btn):
    if btn.value() == 0:
        time.sleep(0.05)
        return btn.value() == 0
    return False


def wait_button_release(btn):
    while btn.value() == 0:
        time.sleep(0.05)


def is_dir(path):
    try:
        return os.stat(path)[0] & 0x4000
    except:
        return False


def file_exists(path):
    try:
        os.stat(path)
        return True
    except:
        return False


def ensure_project_root():
    if is_dir(PROJECT_ROOT):
        return

    try:
        os.mkdir(PROJECT_ROOT)
    except:
        pass


def join_path(folder, filename):
    if folder == "/":
        return "/" + filename
    return folder + "/" + filename


def get_project_folders():
    projects = []
    ensure_project_root()

    try:
        names = os.listdir(PROJECT_ROOT)
    except:
        return projects

    for name in names:
        path = join_path(PROJECT_ROOT, name)

        if is_dir(path):
            main_path = join_path(path, "main.py")

            if file_exists(main_path):
                projects.append(name)

    projects.sort()
    return projects


def display_menu(projects, selected):
    show_line(get_header(), 0, True, False)

    if not projects:
        show_line("No Projects", 1, False, False)
        show_line("Add folder", 2, False, False)
        show_line("with main.py", 3, False, True)
        return

    # Line 0 is header.
    # Lines 1 to 5 are project list.
    visible_rows = 5

    start = 0
    if selected >= visible_rows:
        start = selected - visible_rows + 1

    for row in range(visible_rows):
        idx = start + row

        if idx < len(projects):
            prefix = "> " if idx == selected else "  "
            text = prefix + projects[idx]
        else:
            text = ""

        show_line(text, row + 2, False, False)

    show_line("", visible_rows + 2, False, True)


def display_running(project_name):
    show_line(get_header(), 0, True, False)
    show_line("Running:", 1, False, False)
    show_line(project_name, 2, False, True)


def display_end_program(project_name):
    show_line(get_header(), 0, True, False)
    show_line(project_name, 2, False, False)
    show_line("Program ended", 3, False, False)
    show_line("Press select to exit", 4, False, True)


def display_error(error):
    show_line(get_header(), 0, True, False)
    show_line("ERROR", 1, False, False)
    show_line(str(error)[:20], 2, False, True)


def run_project(project_name):
    main_file = join_path(join_path(PROJECT_ROOT, project_name), "main.py")

    display_running(project_name)
    time.sleep(0.5)

    try:
        with open(main_file) as f:
            code = f.read()

        project_globals = {
            "__name__": "__main__",
            "__file__": main_file,
        }

        exec(code, project_globals)

    except Exception as e:
        display_error(e)

        while True:
            time.sleep(1)


# =========================
# Main launcher
# =========================

def main():
    projects = get_project_folders()

    if not projects:
        display_menu(projects, 0)
        while True:
            time.sleep(1)

    selected = 0
    display_menu(projects, selected)

    while True:
        if button_pressed(left_btn):
            selected -= 1

            if selected < 0:
                selected = len(projects) - 1

            display_menu(projects, selected)
            wait_button_release(left_btn)

        if button_pressed(right_btn):
            selected += 1

            if selected >= len(projects):
                selected = 0

            display_menu(projects, selected)
            wait_button_release(right_btn)

        if button_pressed(select_btn):
            wait_button_release(select_btn)
            run_project(projects[selected])
            display_end_program(projects[selected])
            while not button_pressed(select_btn):
                pass
            display_menu(projects, selected)

        time.sleep(0.05)


main()
