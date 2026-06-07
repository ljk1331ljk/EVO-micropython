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
DOWNLOAD_ORDER_FILE = ".evo_download_order"
BLE_SETTING_ITEM = "__ble_setting__"


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


def read_int_file(path, default=0):
    try:
        with open(path) as f:
            return int(f.read().strip() or default)
    except:
        return default


def project_download_order(project_path):
    order_path = join_path(project_path, DOWNLOAD_ORDER_FILE)
    order = read_int_file(order_path, 0)

    if order:
        return order

    try:
        st = os.stat(join_path(project_path, "main.py"))
        if len(st) > 8:
            return st[8]
    except:
        pass

    return 0


def get_bluetooth_enabled():
    try:
        return bool(evo.get_bluetooth_enabled())
    except:
        try:
            return bool(evo.get_config().get("bluetooth_enabled", True))
        except:
            return True


def set_bluetooth_enabled(on):
    try:
        return bool(evo.set_bluetooth_enabled(on, False))
    except:
        return False


def bluetooth_label():
    if get_bluetooth_enabled():
        return "Bluetooth: On"

    return "Bluetooth: Off"


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
                projects.append((project_download_order(path), name))

    projects.sort(key=lambda item: (-item[0], item[1]))
    return [name for _, name in projects]


def get_menu_items():
    return get_project_folders() + [BLE_SETTING_ITEM]


def menu_item_text(item):
    if item == BLE_SETTING_ITEM:
        return bluetooth_label()

    return item


def display_menu(items, selected):
    show_line(get_header(), 0, True, False)

    # Line 0 is header.
    # Lines 1 to 5 are project list.
    visible_rows = 5

    start = 0
    if selected >= visible_rows:
        start = selected - visible_rows + 1

    for row in range(visible_rows):
        idx = start + row

        if idx < len(items):
            prefix = "> " if idx == selected else "  "
            text = prefix + menu_item_text(items[idx])
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
    project_path = join_path(PROJECT_ROOT, project_name)
    main_file = join_path(project_path, "main.py")
    
    display_running(project_name)
    time.sleep(0.5)

    try:
        import sys

        if project_path not in sys.path:
            sys.path.insert(0, project_path)
        
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
    items = get_menu_items()

    selected = 0
    display_menu(items, selected)

    while True:
        if button_pressed(left_btn):
            selected -= 1

            if selected < 0:
                selected = len(items) - 1

            display_menu(items, selected)
            wait_button_release(left_btn)

        if button_pressed(right_btn):
            selected += 1

            if selected >= len(items):
                selected = 0

            display_menu(items, selected)
            wait_button_release(right_btn)

        if button_pressed(select_btn):
            wait_button_release(select_btn)
            item = items[selected]

            if item == BLE_SETTING_ITEM:
                set_bluetooth_enabled(not get_bluetooth_enabled())
                items = get_menu_items()
                if selected >= len(items):
                    selected = len(items) - 1
                display_menu(items, selected)
            else:
                run_project(item)
                display_end_program(item)
                while not button_pressed(select_btn):
                    pass
                items = get_menu_items()
                if selected >= len(items):
                    selected = len(items) - 1
                display_menu(items, selected)

        time.sleep(0.05)


main()
