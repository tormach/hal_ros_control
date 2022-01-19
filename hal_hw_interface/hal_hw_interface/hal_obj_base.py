"""
:synopsis: Base class for HAL objects.

.. moduleauthor:: John Morris <john@dovetail-automata.com>
"""

import rclpy


class HalObjBase:
    """
    Base class for HAL component objects.

    Takes care of caching objects related to HAL components, like the
    component itself.

    The :py:class:`hal_hw_interface.ros_hal_pin.RosHalComponent` class
    will create the actual HAL component here.  Then pins can access
    the component without it explicitly being passed around.
    """

    _cached_objs = dict()

    @property
    def logger(self):
        return self.node.get_logger()

    @property
    def hal_comp(self):
        """
        Read-only property returning the global HAL component object.

        The HAL component must have already been created by a
        :py:class:`hal_hw_interface.ros_hal_component.RosHalComponent`
        object's :py:func:`init_hal_comp` method.

        :returns: HAL component object
        :rtype: :py:class:`hal.component`
        """
        assert "hal_comp" in self._cached_objs, "`init_hal_comp` not called"
        return self._cached_objs["hal_comp"]

    def init_ros_node(self, args):
        """
        Initialize a new ROS node.

        Sets the following attributes:
        - `node`:  The `rclpy.node.Node` object
        - `node_context`:  The `rclpy.context.Context` object

        Any shutdown callbacks registered with
        :py:func:`add_shutdown_callback` will be run at node shutdown.

        :param args:  Args to ROS node, e.g. `sys.argv`
        :type args:  list
        """
        assert self.compname is not None, "`compname` not set"
        assert "node" not in self._cached_objs
        rclpy.init(args=args)
        co = self._cached_objs
        co["node"] = rclpy.create_node(self.compname)
        co["node_context"] = rclpy.utilities.get_default_context()
        self.node_context.on_shutdown(self._run_shutdown_cbs)
        self.logger.info("Created node")

    @property
    def node(self):
        """
        Read-only property returning the global ROS node object.

        The :py:class:`rclpy.node.Node` must have already been created by a
        :py:class:`hal_hw_interface.ros_hal_component.RosHalComponent`
        object's :py:func:`init_ros_node` method.

        :returns: ROS node object
        :rtype: :py:class:`rclpy.node.Node`
        """
        assert "node" in self._cached_objs, "`init_ros_node` not called"
        return self._cached_objs["node"]

    @property
    def node_context(self):
        """
        Read-only property returning the global ROS node context object.

        The :py:class:`rclpy.context.Context` must have already been
        created by a
        :py:class:`hal_hw_interface.ros_hal_component.RosHalComponent`
        object's :py:func:`init_ros_node` method.

        :returns: ROS node context object
        :rtype: :py:class:`rclpy.context.Context`
        """
        assert "node_context" in self._cached_objs, "`init_ros_node` not called"
        return self._cached_objs["node_context"]

    def get_ros_param(self, name, default=None):
        """
        Retrieve a ROS parameter value.

        This is shorthand for declaring, if needed, and retrieving the
        value of a ROS param.

        :param name: ROS parameter name
        :type name: str
        :param default: Default value if key not on param server
        :type default: any
        :returns: parameter value
        :rtype: XmlRpcLegalValue
        """
        # Params must be declared once and only once, so cache
        # declarations.
        if name not in self._cached_objs.setdefault("rosparam_decls", dict()):
            self.logger.info(f"New param {name}, default {repr(default)}")
            decl = self.node.declare_parameter(name, default)
            self._cached_objs["rosparam_decls"][name] = decl
        return self._cached_objs["rosparam_decls"][name].value

    def add_shutdown_callback(self, cb, prio=500):
        """
        Add a shutdown callback.

        Add a prioritized callback performing some extra shutdown
        action (such as disconnect from a service) that will be run
        from
        :py:func:`hal_hw_interface.ros_hal_component.RosHalComponent.shutdown_component`
        at component shut down time.  Callbacks are executed in
        priority order from low to high.

        :param cb: Callback function
        :type cb: callable
        :param prio: Priority, default 500
        :type prio: int
        """
        assert callable(cb), "shutdown callback must be callable"
        shutdown_cbs = self._cached_objs.setdefault("shutdown_cbs", {})
        while prio in shutdown_cbs:
            prio += 0.1
        shutdown_cbs[prio] = cb

    def _run_shutdown_cbs(self):
        cb_map = self._cached_objs.setdefault("shutdown_cbs", {})
        if not cb_map:
            self.logger.info("No more shutdown cbs to run")
            return  # Already run, or none added
        while cb_map:
            top_cb_key = list(cb_map.keys())[0]
            self.logger.info(f"Running shutdown cb {top_cb_key}")
            cb_map.pop(top_cb_key)()
