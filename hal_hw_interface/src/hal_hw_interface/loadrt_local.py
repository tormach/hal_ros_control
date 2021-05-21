import rospy
import os
from machinekit import rtapi, hal


def loadrt_local(modname):
    """Load a locally-built HAL component not installed in the standard
    module directory
    """
    if modname in hal.components:
        return
    for path in os.environ.get('LD_LIBRARY_PATH', '').split(':'):
        rospy.logdebug(f"Checking for {modname}.so in {path}")
        modpath = os.path.join(path, f'{modname}')
        if os.path.exists(f'{modpath}.so'):
            break
    else:
        raise RuntimeError(f'Unable to locate {modname} module')
    rtapi.loadrt(modpath)
