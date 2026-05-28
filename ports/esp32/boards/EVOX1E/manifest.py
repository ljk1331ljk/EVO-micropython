# Freeze standard ESP32 port modules (optional but recommended)
include("$(PORT_DIR)/boards/manifest.py")

# Freeze custom Evo modules
freeze("$(PORT_DIR)/frozen/evo")
freeze("$(BOARD_DIR)", "boot.py")
module("pins.py", base_path=".")