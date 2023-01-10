import rclpy
import os
from machinekit.hal.cyruntime import rtapi
import machinekit.hal.pyhal as hal

def loadrt_local(modname):
    """
    Load a RT HAL component.

    Load a locally-built HAL component installed outside standard module
    directories.  Uses the `LD_LIBRARY_PATH` environment variable as a
    search path.
    """
    if modname in hal.components:
        return
    logger = rclpy.logging.get_logger("loadrt_local")
    for path in os.environ.get("LD_LIBRARY_PATH", "").split(":"):
        logger.debug(f"Checking for {modname}.so in {path}")
        modpath = os.path.join(path, f"{modname}")
        if os.path.exists(f"{modpath}.so"):
            break
    else:
        raise RuntimeError(f"Unable to locate {modname} module")
    rtapi.loadrt(modpath)
