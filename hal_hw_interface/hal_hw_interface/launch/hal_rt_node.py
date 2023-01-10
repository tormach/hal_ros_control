import os
from typing import Optional

from launch import SomeSubstitutionsType, logging
from launch.utilities import perform_substitutions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

from machinekit.hal.cyruntime import rtapi
import machinekit.hal.cyhal as hal
from .hal_ordered_action import HalOrderedNode, HalThreadedReadyAction


class HalRTNode(HalOrderedNode, HalThreadedReadyAction):
    """Action that loads a HAL RT component ROS node."""

    def __init__(
        self,
        *,
        component: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        hal_name=None,
        **kwargs,
    ) -> None:
        """
        Construct a HalRTNode action.

        Similar to :class:`launch.actions.Node`, this action launches a
        ROS node, but as a real time HAL component.
        """
        if package is not None:
            pkg_prefix = FindPackagePrefix(package=package)
            path = PathJoinSubstitution([pkg_prefix, "lib", component])
        else:
            path = component
        hal_name = hal_name or os.path.basename(component)

        # Pretend the component (a loadable plugin) is the full path
        # of an executable, thereby forced into the executable `Node`
        # scheme.  "Forced," for sure.
        super().__init__(
            hal_name=hal_name, package=None, executable=path, **kwargs
        )
        self.__package = package
        self.__component = component
        self.__component_path = path
        self.__logger = logging.get_logger(f"{__name__}({self.hal_name})")

    def is_ready(self, context):
        # If `loadrt()` call hasn't returned, not ready
        if not super().is_ready(context):
            return False
        # Check if component is registered and become ready
        if self.hal_name not in hal.components:
            self.__logger.debug(
                "...HAL comp {self.hal_name} not yet registered"
            )
            return False
        elif hal.components[self.hal_name].state != hal.COMP_READY:
            self.__logger.debug("...HAL comp not yet ready")
            return False
        else:
            return True

    def execute_deferred_cb(self, context):
        # Expand command substitutions
        cmd = [perform_substitutions(context, c) for c in self.cmd]
        comp_path = cmd[0]
        self.__logger.info(f"Loading HAL RT component {comp_path}")
        self.comp_args = ",".join(cmd[1:])
        self.__logger.info(f"  args: ARGV={self.comp_args}")

        rtapi.loadrt(comp_path, ARGV=self.comp_args)

        self.__logger.info("loadrt complete")
