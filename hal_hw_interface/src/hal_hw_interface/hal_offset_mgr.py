# -*- coding: utf-8 -*-
import rospy
import attr
from hal_hw_interface.redis_store_hal_pin import RedisStoreHalPin
from hal_hw_interface.ros_hal_pin import RosHalPin
from hal_hw_interface.ros_hal_component import (
    RosHalComponent,
    HalHWInterfaceException,
)


@attr.s
class HalOffsetMgrPin(RedisStoreHalPin):
    '''Represents the offset of one joint.

    By default, the pin name is :code:`<name>_offset`, where
    :code:`<name>` might simply be a joint name.  This pin connects to
    an :code:`offset` comp's :code:`offset` pin.  Its value is
    persisted in a redis store key
    :code:`hal_offset_mgr/<name>_offset`.

    A second, complementary :code:`<name>_fb-in` input pin connects to
    the :code:`offset` comp :code:`fb-in` pin.  The
    :code:`zero_joint()` function copies this value to the offset pin
    and updates the redis store.

    :param name: The HAL pin name prefix
    :type name: str
    :param hal_type: HAL pin data type, one of :code:`['BIT', 'U32',
      'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of :code:`['IN', 'OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    '''

    _default_hal_type = 'FLOAT'
    _default_hal_dir = 'OUT'

    offset_pin_name = attr.ib()
    fb_in_pin_name = attr.ib()

    # Attribute default factories
    @offset_pin_name.default
    def _offset_pin_name_default(self):
        return self.name + '_offset'

    @fb_in_pin_name.default
    def _fb_in_pin_name_default(self):
        return self.name + '_fb-in'

    @property
    def pin_name(self):
        return self.offset_pin_name

    def _hal_init(self):
        # Set up the offset pin
        super(HalOffsetMgrPin, self)._hal_init()

        # Set up fb-in pin
        self.hal_fb_in_pin = RosHalPin(self.fb_in_pin_name, self.hal_type, 'IN')

        rospy.loginfo(
            'Created {} pin "{}" type "{}"'.format(
                self.hal_dir, self.fb_in_pin_name, self.hal_type
            )
        )

    def zero_joint(self):
        '''Zero the offset by copying the :code:`fb-in` pin to the
        :code:`offset` pin and to redis
        '''
        new_offset = self.hal_fb_in_pin.get_pin()
        self.set_pin(new_offset)
        self.set_redis_from_pin()
        return new_offset

    def load_offset(self):
        '''Load the offset value from redis and copy to the :code:`offset` pin
        '''
        self.set_pin_from_redis(default=0)
        value = self.get_pin()  # Re-read pin for feedback
        rospy.loginfo("Restored %s offset to %.4f", self.pin_name, value)
        return value


class HalOffsetMgr(RosHalComponent):
    '''HAL user component managing zero offsets that persist across
    machine restarts

    It reads joint names from the ROS parameter server, and creates
    two HAL pins for each:  :code:`hal_offset_mgr.<joint>_offset` and
    :code:`hal_offset_mgr.<joint>_fb-in`, meant to be attached to
    corresponding pins of the joint's :code:`offset` component.

    The machine start-up procedure should pull the
    :code:`hal_offset_mgr.load_params` command pin high.  The
    component will then copy saved offset values from the redis
    :code:`hal_offset_mgr/<joint>_offset` keys to the corresponding
    :code:`hal_offset_mgr.<joint>_offset` pins for each joint, and pull
    :code:`hal_offset_mgr.load_params` low again.

    The user interface should present the user with a 'zero offsets'
    switch that requests all offsets zeroed at the current position by
    pulling the :code:`hal_offset_mgr.zero_all_joints` pin high.  The
    :code:`hal_offset_mgr.enable` pin must be low, or the component
    will refuse to do anything and issue an error message to that
    effect.  Otherwise, the component will zero offsets by copying the
    :code:`hal_offset_mgr.<joint>_fb-in` pins' values to the
    :code:`hal_offset_mgr.<joint>_offset` pins and the redis
    :code:`hal_offset_mgr/<joint>_offset` keys for each joint.
    '''

    compname = 'hal_offset_mgr'

    def setup_component(self):
        '''Read joints from ROS parameter server and create component pins
        '''

        # get joint info from hardware_interface config
        joint_names = rospy.get_param('hardware_interface/joints', None)
        if joint_names is None:
            raise HalHWInterfaceException(
                "Error reading ROS param hardware_interface/joints"
            )

        # create joint offset pins
        self.offset_pins = []
        for name in joint_names:
            o = HalOffsetMgrPin(name)
            self.offset_pins.append(o)

        # create enable and zero + load command pins
        self.zero_all_joints = RosHalPin('zero_all_joints', 'BIT', 'IO')
        self.load_params = RosHalPin('load_params', 'BIT', 'IO')
        self.enable = RosHalPin('enable', 'BIT', 'IN')

    def update(self):
        '''Periodic update function; watches for and executes
        :code:`load_params` or :code:`zero_all_joints` requests
        '''

        if self.load_params.get_pin():
            # Command:  Load initial offsets from stored parameters
            for p in self.offset_pins:
                p.load_offset()
            self.load_params.set_pin(False)
            return

        if self.zero_all_joints.get_pin():
            # Command:  Zero all joints
            if self.enable.get_pin():
                rospy.logerr("Not zeroing joints while enabled")
                self.zero_all_joints.set_pin(False)
                return

            for p in self.offset_pins:
                p.zero_joint()

            self.zero_all_joints.set_pin(False)
            rospy.loginfo("All joints zeroed")

    def shutdown_component(self):
        '''Close redis client connection at shutdown
        '''
        RedisStoreHalPin.shutdown()
