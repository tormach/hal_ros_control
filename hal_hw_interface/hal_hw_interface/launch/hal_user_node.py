import os
import machinekit.hal.cyhal as hal
from launch import logging
from .hal_ordered_action import HalOrderedNode, HalAsyncReadyAction


class HalUserNode(HalOrderedNode, HalAsyncReadyAction):
    """
    Action that loads a HAL "user" (non-RT) component ROS node.

    The component isn't started until after `execution_deferred()`
    returns, and it may need time before the HAL component is marked
    ready.
    """

    def __init__(self, *, executable, hal_name=None, **kwargs) -> None:
        """
        Construct a HalUserNode action.

        :param: hal_name After starting, wait until the HAL component
            named by `hal_name` comes ready; defaults to basename of
            the `executable` param
        """
        self.__logger = logging.get_logger(__name__)
        hal_name = hal_name or os.path.basename(executable)
        super().__init__(hal_name=hal_name, executable=executable, **kwargs)
        self.__executable = executable

    def is_ready(self, context):
        # Check if component is registered and become ready
        if self.hal_name not in hal.components:
            self.__logger.debug(
                "...HAL comp {self.hal_name} not yet registered"
            )
            return False
        elif hal.components[self.hal_name].state != hal.COMP_READY:
            self.__logger.debug("...HAL comp {self.hal_name} not yet ready")
            return False
        else:
            return True
