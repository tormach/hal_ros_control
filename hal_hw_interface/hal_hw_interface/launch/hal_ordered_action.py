from threading import Thread

from launch import logging
from launch.action import Action
from launch.actions import (
    OpaqueFunction,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.event_handler import EventHandler
from launch.events import Shutdown

from launch_ros.actions import Node

from machinekit import rtapi
from .hal_ready import HalReady


class HalOrderedAction(Action):
    """
    Mixin for deferred actions.

    This action is deferred until a HalReady event named by
    `triggered_by`, and after execution emits a HalReady event named
    by `triggers`.

    This mixin class must precede the base class in subclass
    definitions, since it overrides the `execute()` method.

    Subclasses must do the actual work normally done by `execute()` in
    the `execute_deferred()` method instead.
    """

    # By default, don't pass `name` param to superclass __init__()
    pass_name = False

    def __init__(self, *, hal_name=None, **kwargs):
        super().__init__(**kwargs)
        self.__logger = logging.get_logger(__name__)
        self.__canceled = False
        self.__hal_name = hal_name
        assert self.__hal_name

    @property
    def hal_name(self):
        return self.__hal_name

    def cancel(self, context, warn=True):
        """
        Cancel this Action's handlers.

        This function is not thread-safe and should be called only from
        under another coroutine.
        """
        self.__canceled = True
        if hasattr(self, "handler"):
            msg = f"Canceling handler {self}"
            getattr(self.__logger, "warn" if warn else "debug")(msg)
            try:
                context.unregister_event_handler(self.handler)
            except Exception:
                pass

    def set_triggers(self, *, triggered_by, triggers):
        self.__logger.debug(
            f"{self}:  triggered_by {triggered_by}; triggers {triggers}"
        )
        self.triggered_by = triggered_by
        self.triggers = triggers

    def init_rtapi(self):
        # Be sure rtapi is initialized
        if not getattr(rtapi, "__rtapicmd", False):
            rtapi.init_RTAPI()
        self.rtapi = rtapi

    def execute_deferred(self, context):
        """Execute the action."""
        self.__logger.info(f"Executing deferred action {self}")
        return self.__execute(context)

    def ready_event(self):
        """Return `HalReady` event to trigger next Action."""
        if self.triggers:
            return [EmitEvent(event=HalReady(self.triggers))]
        else:
            return []

    def shutdown_action(self, reason):
        loginfo = LogInfo(msg=f"Shutting down:  {reason}")
        shutdown = Shutdown(reason=reason, due_to_sigint=False)
        return [loginfo, EmitEvent(event=shutdown)]

    def on_ready_event(self, context):
        """
        `HalReady` event handler for executing deferred action.

        Do the following:

        - Finalize command arguments
        - Initialize the `rtapi` object
        - Cancel the event handler
        - Add back locals from original execute() call
        - Execute the deferred action
        - Trigger the next action
        """
        if context.is_shutdown:
            # If shutdown starts before execution can start, don't
            # start execution.
            self.__logger.warn(f"HAL action {self} canceled before execution")
            return None

        self.init_rtapi()
        self.cancel(context, warn=False)
        context.extend_locals(self.__context_locals)
        try:
            res = self.execute_deferred(context)
        except Exception as e:
            ecls, emsg = (e.__class__.__name__, str(e))
            return self.shutdown_action(
                f"{self} shutdown on error:  {ecls}: {emsg}"
            )
        return (res or []) + self.ready_event()

    def matcher(self, event):
        """Match `HalReady` event corresponding to this action."""
        return isinstance(event, HalReady) and event.name == self.triggered_by

    def execute(self, context):
        """
        Defer action execution.

        - Create a task for the coroutine that waits until event or
          canceled

        - Coroutine asynchronously fires this action after previous
          actions complete, unless canceled first.
        """
        assert hasattr(self, "triggered_by"), f"{self}: set_triggers() uncalled"

        if context.is_shutdown:
            # If shutdown starts before execution can start, don't
            # start execution.
            return None

        # No `triggered_by` name means ready NOW, just do it.
        if self.triggered_by is None:
            return self.on_ready_event(context)

        if self.__canceled:
            self.__logger.warn(f"HAL action {self} canceled before execution")
            return

        self.handler = EventHandler(
            matcher=self.matcher,
            entities=OpaqueFunction(function=self.on_ready_event),
        )
        context.register_event_handler(self.handler)

        # Cancel handlers at the 'shutdown' event so as to not hold up
        # the launch process
        context.register_event_handler(
            EventHandler(
                matcher=lambda event: isinstance(event, Shutdown),
                entities=OpaqueFunction(function=self.cancel),
            )
        )

        # Save the execute() method & context locals we would have run
        self.__execute = super().execute
        self.__context_locals = context.get_locals_as_dict()

    def __repr__(self):
        return f"<{self.__class__.__name__} '{self.hal_name}'>"


class HalOrderedNode(Node, HalOrderedAction, ExecuteProcess):
    """
    Class for ordered HAL Nodes.

    This class inserts :class:`HalOrderedAction` between
    :class:`launch_ros.actions.Node` and
    :class:`launch.actions.ExecuteProcess` in order to hijack the call
    to the latter's :meth:`execute` method.

    This lets us get away with subclassing
    :class:`launch_ros.actions.node.Node` and reusing its `execute()`
    method, which contains non-optional logic that uses munged/private
    parameters and uses the deprecated (??!?) `__node_name` attribute.
    """

    # Node.__init__() uses `name`
    pass_name = True

    def __init__(self, **kwargs) -> None:
        kwargs.setdefault("on_exit", self.shutdown_action("Exited"))
        super().__init__(**kwargs)


class HalAsyncReadyAction(HalOrderedAction):
    """
    HAL action with deferred HalReady event.

    For actions that aren't ready immediately after
    `execution_deferred()` returns, this action periodically checks
    whether `is_ready() == True` before triggering next action.  If
    still not ready before a timeout, it aborts launch.
    """

    # Ready check interval in seconds
    ready_check_interval = 0.1

    def __init__(self, wait_timeout=5.0, **kwargs) -> None:
        """
        Construct a HalAsyncReadyAction action.

        :param: wait_timeout After starting, if component does not
            become ready within this number of seconds, abort the
            launch.  Defaults to 5.0.
        """
        super().__init__(**kwargs)
        self.__logger = logging.get_logger(__name__)
        self.__wait_timeout = wait_timeout
        self.__ready_checker = None  # ready check timer
        self.__ready_timeout = None  # wait_timeout timer

    def is_ready(self, context):
        raise NotImplementedError("Subclasses must implement is_ready()")

    def check_ready(self, context):
        # Check if component is registered and become ready
        if self.is_ready(context):
            # Component ready; cancel timers and trigger next event
            self.__ready_timeout.cancel()
            self.__logger.info(f"HAL action {self.hal_name} ready")
            return [EmitEvent(event=HalReady(self.triggers))]
        else:
            # Not ready; schedule another check
            return [
                TimerAction(
                    period=self.ready_check_interval,
                    actions=[OpaqueFunction(function=self.check_ready)],
                ),
            ]

    def cancel(self, context, warn=True):
        for t in [self.__ready_timeout, self.__ready_checker]:
            if t is not None:
                t.cancel()
        super().cancel(context, warn=warn)

    def ready_event(self):
        if not self.triggers:
            return []
        # Periodic ready check timer:  trigger next action when comp is ready
        self.__ready_checker = TimerAction(
            period=self.ready_check_interval,
            actions=[OpaqueFunction(function=self.check_ready)],
        )
        # Wait timeout timer:  shut down launch
        self.__ready_timeout = TimerAction(
            period=self.__wait_timeout,
            actions=self.shutdown_action(
                f"HAL action {self.hal_name} ready wait timed out"
            ),
        )
        return [self.__ready_timeout, self.__ready_checker]


class HalThreadedReadyAction(HalAsyncReadyAction):
    """
    HAL action running in a thread with deferred HalReady event.

    An Action that blocks for too long and isn't suited to run directly in a
    coroutine can instead be run in a thread.  As a subclass of
    :class:`HalAsyncReadyAction`, this class emits a deferred HalReady event.

    Subclasses must implement :meth:`execute_deferred_cb` that executes the
    action in a thread.

    Subclass :meth:`is_ready` method may check this class's :meth:`is_ready` if
    callback exit is a condition of being ready.
    """

    def __init__(self, **kwargs) -> None:
        """Construct a HalThreadedReadyAction action."""
        super().__init__(**kwargs)
        self.__logger = logging.get_logger(__name__)

    def execute_deferred_cb(self, context):
        raise NotImplementedError(
            "Subclasses must implement execute_deferred_cb()"
        )

    def execute_deferred(self, context):
        """Execute the action in a thread."""
        self.__thread = Thread(
            name=self.hal_name, target=self.execute_deferred_cb, args=(context,)
        )
        self.__joined = False
        self.__thread.start()

    def check_thread_exit(self):
        if self.__thread.is_alive():
            return False
        else:
            if not self.__joined:
                self.__logger.info(f"HAL action {self.hal_name} thread exited")
                self.__thread.join()
                self.__joined = True
            return True

    def is_ready(self, context):
        # If callback hasn't returned, not ready
        return self.check_thread_exit()

    def check_ready(self, context):
        # For subclasses is_ready() that don't call super().is_ready(), check
        # thread joined
        self.check_thread_exit()
        return super().check_ready(context)
