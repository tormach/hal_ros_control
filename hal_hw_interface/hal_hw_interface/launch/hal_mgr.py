import functools

from launch import logging
from launch.actions import LogInfo, EmitEvent
from launch.events import Shutdown
from launch_ros.actions.node import Node
from launch_ros.ros_adapters import get_ros_node

from std_msgs.msg import Bool

from .hal_ready import HalReady


class HalMgr(Node):
    """Action that starts Machinekit HAL with the `hal_mgr` ROS node."""

    def __init__(
        self,
        *,
        package="hal_hw_interface",
        executable="hal_mgr",
        triggers=None,
        ready_timeout=10,
        on_exit=None,
        **kwargs,
    ) -> None:
        """
        Construct a HalMgr action.

        Launch the `hal_mgr` node as in :class:`launch.actions.Node`.
        When the `hal_mgr/ready` topic is `True`, emit a
        :class:`hal_hw_interface.launch.hal_ready_event.HalReady`
        event.
        """
        self.__logger = logging.get_logger(__name__)
        self.triggers = triggers
        if on_exit is None:
            on_exit = self.shutdown_action("Exited")

        super().__init__(
            package=package, executable=executable, on_exit=on_exit, **kwargs
        )

    def shutdown_action(self, reason):
        # On exit, shut down launch
        loginfo = LogInfo(msg=f"hal_mgr shutting down:  {reason}")
        shutdown = Shutdown(reason=reason, due_to_sigint=False)
        return [loginfo, EmitEvent(event=shutdown)]

    def _on_ready_event(self, context, msg):
        # If the hal_mgr/ready topic is True, emit an event; no more
        # to do, so destroy the subscriber
        if msg.data:
            context.asyncio_loop.call_soon_threadsafe(
                lambda: context.emit_event_sync(HalReady(self.triggers))
            )
            self.__hal_mgr_ready_subscription.destroy()

    def execute(self, context):
        """Execute the action."""
        # Start hal_mgr
        res = super().execute(context)

        # Create a subscription to monitor when RTAPI/HAL is running
        node = get_ros_node(context)
        self.__hal_mgr_ready_subscription = node.create_subscription(
            Bool,
            "hal_mgr/ready",
            functools.partial(self._on_ready_event, context),
            10,
        )

        return res
