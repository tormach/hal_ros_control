# -*- coding: utf-8 -*-

"""
   :synopsis: Boilerplate for HAL user components in ROS nodes

   .. moduleauthor:: John Morris <john@dovetail-automata.com>
"""

import rospy
from hal_hw_interface.hal_obj_base import HalObjBase
from hal_hw_interface.exception import HalHWInterfaceException


class RosHalComponent(HalObjBase):
    """Base class implementing a HAL user component in a ROS node

    This class provides the skeleton of a ROS node that runs a
    Machinekit HAL user component.  It does the tedious part of
    setting up the node and the component, running it, and shutting it
    down.

    These components will often serve to connect the component's HAL
    pins (which are connected in a larger HAL configuration from the
    `hal_mgr` ROS node) to ROS publishers, subscribers, or services.
    The :py:class:`hal_hw_interface.ros_hal_pin.RosHalPin` classes provide an
    easy-to-use interface to set up these HAL pins connected to ROS.

    Basic usage:

    * Create a subclass in a new module
    * Name the component by defining :py:attr:`compname`
    * Set up HAL pins and ROS communication in
      :py:func:`setup_component`
    * Handle periodic pin and ROS update logic in :py:func:`update`

    A bare-bones example:

    .. code-block:: python

        from hal_hw_interface.ros_hal_component import RosHalComponent

        class MyRosHalComp(RosHalComponent):
            compname = 'my_comp'

            def setup_component(self):
                # Set up ROS HAL pins

            def update(self):
                # Do any updates, reading and setting pins
    """

    # Override in subclasses
    compname = None
    """The name of the HAL component

    This will also be used as a default prefix for ROS names.
    """

    def __init__(self):
        if self.compname is None:
            raise NotImplementedError(
                "Subclasses must set 'compname' class attribute"
            )

        # Create ROS node
        rospy.init_node(self.compname)
        rospy.loginfo("Initializing '%s' component" % self.compname)

        # Publisher update rate in Hz
        self.update_rate = self.get_ros_param('update_rate', 10)
        self.rate = rospy.Rate(self.update_rate)
        rospy.logdebug("Publish update rate = %.1f" % self.update_rate)

        # Init HAL component
        self.init_hal_comp()

        # Let the submodule initialize the HAL component
        self.setup_component()

        # Finish initialization
        self.hal_comp.ready()
        rospy.loginfo("User component '%s' ready" % self.compname)

    def setup_component(self):
        """Set up the ROS node and HAL component

        This MUST be defined in subclasses to perform all set up tasks
        for the ROS node and HAL component.  Much of this is automated
        in the :py:mod:`hal_hw_interface.ros_hal_pin` classes,
        which create HAL pins attached to ROS subscribers, publishers
        and services.
        """
        raise NotImplementedError(
            "Subclasses must define 'setup_component' method"
        )

    def run(self):
        """Run the ROS node/HAL component

        Runs the component, calling the :py:func:`update` function in
        a loop until shutdown.

        The :py:attr:`rospy.Rate` update rate will be taken from the
        ROS parameter `<compname>/update_rate`, defaulting to 10 Hz.
        """
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

    def update(self):
        """The ROS node and HAL component update routine

        This MUST be defined in subclasses.  It is run periodically
        from the :py:func:`run` function, and should take care of all
        updates, such as reading or writing pins and communication
        with ROS.
        """
        # Override in subclasses
        raise NotImplementedError("Subclasses must define 'update' method")

    def shutdown_component(self):
        """Perform extra shutdown actions

        Executes the list of callbacks defined by calls to
        :py:func:`hal_hw_interface.hal_obj_base.HalObjBase.add_shutdown_callback`.
        """
        for cb in self._cached_objs.setdefault('shutdown_cbs', []):
            cb()

    def main(self):
        """The ROS node and HAL component `main()` function

        The ROS node executable python script should define or import
        the component class, and then initialize and run it like
        this:

        .. code-block:: python

            #!/usr/bin/env python

            # Import or define MyRosHalComp class

            if __name__ == '__main__':
                MyRosHalComp().main()

        Then start the script from your `hal_mgr` `.hal` file::

            loadusr -Wn my_comp rosrun my_ros_hal_pkg my_ros_hal_comp
        """
        try:
            self.run()
        except rospy.ROSInterruptException:
            pass
        except HalHWInterfaceException as e:
            rospy.logfatal(e.message)
