.. hal_hw_interface documentation master file, created by
   sphinx-quickstart on Sun Feb 10 14:11:52 2019.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Documentation for :code:`hal_hw_interface`
============================================

This ROS package provides interfaces to Machinekit HAL.  It consists
of several main parts:  a real-time joint controller based on
``ros_control``, a Python interface for writing HAL components with
pins that connect to ROS, a HAL manager node for setting up and
tearing down the HAL threading environment, and a simple,
fully-functioning ``hal_rrbot_control`` example to demonstrate a basic
``hal_hw_interface`` robot integration.

* :Real-time HAL ros_control component:

    The :code:`hal_hw_interface` HAL component implements the
    ``ros_control`` |RobotHW|_ class via |GenericHWInterface|_ to move
    joints in real-time.  See the `C++ documentation`_.

* :Python HAL interface:

    The Python :py:mod:`hal_hw_interface` modules provide classes for
    easy interfacing of ROS and HAL with (non-real-time) Python code.

  * :Python HAL components:

      The
      :py:class:`hal_hw_interface.ros_hal_component.RosHalComponent`
      class takes care of boilerplate code for initializing Python ROS
      nodes and HAL components.  Subclass this for your component and
      add ROS-connected HAL pins with start, update and shutdown
      logic.

  * :Python HAL pins:

      Add HAL pins to your Python HAL component with the
      :py:class:`hal_hw_interface.ros_hal_pin.RosHalPin` class.  Its
      subclasses communicate their pin values with ROS through
      publishing or subscribing to a ROS topic, or running a ROS
      service.  A special
      :py:class:`hal_hw_interface.redis_store_hal_pin.RedisStoreHalPin`
      class persists its value in a redis kel-value store between
      application restarts.

  * :Out of the box Python HAL components:

      The :py:mod:`hal_hw_interface.hal_io_comp` and
      :py:mod:`hal_hw_interface.hal_offset_mgr` modules implement ROS
      HAL components for connecting robot I/O and for managing joint
      offsets, respectively.  These are ready to use as-is from your
      application, or serve as simple examples of how to write your
      own HAL components.

* :HAL manager node:

    The `hal_mgr` node takes care of starting and stopping Machinekit
    HAL within the context of a ROS node started by :code:`roslaunch`.

    .. todo::  Document the HAL manager node.

* :The ``hal_rrbot_control`` example:

    The :code:`hal_rrbot_control` ROS package represents a simplistic
    but complete, working example of a HAL-controlled robot with
    joints connected through :code:`ros_control` and a gripper
    connected through a ROS service.

    .. todo::  Document the :code:`hal_rrbot_control` example

.. Recursive formatting not allowed:
     http://docutils.sourceforge.net/FAQ.html#is-nested-inline-markup-possible

.. _C++ documentation: c++/index.html

.. |RobotHW| replace:: :code:`hardware_interface::RobotHW`
.. _RobotHW:  http://wiki.ros.org/hardware_interface

.. |GenericHWInterface| replace:: :code:`ros_control_boilerplate::GenericHWInterface`
.. _GenericHWInterface:  http://wiki.ros.org/ros_control_boilerplate

.. |hal_hw_interface| replace:: :code:`hardware_interface`
.. _hal_hw_interface:  http://wiki.ros.org/hardware_interface

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
