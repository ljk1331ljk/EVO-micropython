from evo import *

evo_init(start_download=False)

try:
    if get_bluetooth_enabled():
        cfg = get_download_config()
        if cfg.get("start_on_boot", False):
            import EvoDownloadManager
            EvoDownloadManager.start()
except Exception as e:
    print("EvoDownloadManager failed:", e)
