from evo import *

evo_init()

try:
    import os
    os.stat("/main.py")
except OSError:
    try:
        import EvoDefaultMain
    except Exception as e:
        print("Default program failed:", e)
