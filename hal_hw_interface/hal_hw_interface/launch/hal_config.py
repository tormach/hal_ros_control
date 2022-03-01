from launch.actions import GroupAction

from .hal_ordered_action import HalOrderedAction
from .hal_mgr import HalMgr


class HalConfig(GroupAction):
    """
    Action that starts Machinekit HAL and loads a HAL configuration.

    This group action starts the `hal_mgr` node.  Once it has brought
    up Machinekit HAL, any deferred `HalRTNode` or `HalNode` actions
    will execute, loading their respective HAL components.  Once those
    are all ready, a deferred `HalConfig` action will signal the
    `hal_mgr` to load the configured HAL file.

    This action is necessary for two main reasons:

    - Traditional HAL file configuration for loading components can't
      encompass the full complexity of ROS node launch configuration.
      This scheme separates out the loading of HAL components running
      ROS nodes into familiar ROS launch actions, while leaving the
      more standard portion of the HAL configuration its usual HAL
      file form.

    - Machinekit HAL startup sequence requires the RTAPI/HAL
      environment start before the HAL configuration can load, and in
      the HAL configuration, HAL components must be loaded before pins
      may be netted.  This action enforces that sequence.
    """

    def __init__(
        self,
        actions,
        *,
        scoped=True,
        launch_configurations=None,
        **hal_mgr_kwargs,
    ):
        """
        Create the HalConfig action.

        This action passes all the usual keyword args to the `hal_mgr`
        node constructor.

        The `actions` arg is a list of `HalRTNode`, `HalNode` and
        `HalConfig` actions; the `hal_mgr` node will be inserted at
        the beginning.  The `actions` and `launch_configurations` args
        are used as arguments to the `GroupAction`.
        """
        self.actions = actions

        # Create the hal_mgr node action
        hal_mgr_kwargs.setdefault("name", "hal_mgr")
        triggers = (actions[0].hal_name if actions else None) or "action0"
        hal_mgr_kwargs.setdefault("triggers", triggers)
        hal_mgr_action = HalMgr(**hal_mgr_kwargs)

        # Add ordering to actions
        for i, action in enumerate(actions):
            # This GroupAction only deals with HalOrderedActions
            assert isinstance(
                action, HalOrderedAction
            ), "HalConfig actions must be HalOrderedAction subclasses"
            # This action is triggered_by previous action's triggers
            triggered_by = triggers
            # Make up a trigger name for the next action
            if len(actions) - i == 1:  # Last action in list
                triggers = None
            else:
                triggers = actions[i + 1].hal_name or f"action{i}"
            # Set ordering for this action
            action.set_triggers(
                triggered_by=triggered_by,
                triggers=triggers,
            )

        # Create the group action
        actions.insert(0, hal_mgr_action)
        super().__init__(
            actions, scoped=scoped, launch_configurations=launch_configurations
        )
