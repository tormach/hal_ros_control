# -*- coding: utf-8 -*-

"""
   :synopsis: ROS-connected HAL pin objects for use in
     :py:mod:`hal_hw_interface.ros_hal_component`

   .. moduleauthor:: John Morris <john@dovetail-automata.com>

   .. inheritance-diagram::
        hal_hw_interface.ros_hal_pin.RosHalPin
        hal_hw_interface.ros_hal_pin.RosHalPinPublisher
        hal_hw_interface.ros_hal_pin.RosHalPinSubscriber
        hal_hw_interface.ros_hal_pin.RosHalPinService
"""

import sys
import attr
import rospy
from hal_hw_interface.hal_obj_base import HalObjBase
from hal_hw_interface.exception import HalHWInterfaceException
from hal_hw_interface.hal_pin_attrs import HalPinDir, HalPinType
from std_msgs.msg import Bool, Float64, UInt32, Int32
from std_srvs.srv import SetBool
from hal_hw_interface.srv import SetUInt32, SetInt32, SetFloat64
from math import isclose


@attr.s
class RosHalPin(HalObjBase):
    '''Basic HAL pin for use in
    :py:class:`hal_hw_interface.ros_hal_component.RosHalComponent`
    user components

    This is a basic HAL pin with no special connection to ROS.  It may
    be used in a HAL component, and serves as a base class for other
    pins.

    :param name: The HAL pin name
    :type name: str
    :param hal_type: HAL pin data type, one of :code:`['BIT', 'U32',
      'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of :code:`['IN', 'OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    '''

    _default_hal_type = None
    _default_hal_dir = 'IN'  # Subclasses may override for hal_dir attribute

    name = attr.ib()
    hal_type = attr.ib(converter=HalPinType)
    hal_dir = attr.ib(converter=HalPinDir)

    # Attribute default factories
    @hal_dir.default
    def _hal_dir_default(self):
        return HalPinDir(self._default_hal_dir)

    @hal_type.default
    def _hal_type_default(self):
        if self._default_hal_type is None:
            raise TypeError('%s requires hal_type= argument' % self.__class__)
        return HalPinType(self._default_hal_type)

    @property
    def pin_name(self):
        '''Return the pin_name; read-only property

        In some subclasses, this may not be the same as the
        :code:`name` parameter supplied to the constructor.

        :returns:  :py:class:`str` pin name
        '''
        return self.name

    def __attrs_post_init__(self):
        self._hal_init()
        self._ros_init()

    def _hal_init(self):
        self.hal_pin = self.hal_comp.newpin(
            self.pin_name, self.hal_type, self.hal_dir
        )

        rospy.loginfo(
            'Created pin "{}" type/dir "{}/{}"'.format(
                self.pin_name, self.hal_type, self.hal_dir
            )
        )

    def _ros_init(self):
        # May be implemented in subclasses
        pass

    def update(self):
        '''An update function; used in some subclasses
        '''
        # May be implemented in subclasses
        raise NotImplementedError()

    def set_pin(self, value):
        """Set the HAL pin's value

        Does not apply to pins initialized with :code:`hal_dir='IN'`.

        :param value: The value to set
        """
        self.hal_comp[self.pin_name] = value

    def get_pin(self):
        """Return the HAL pin's value

        :returns: HAL pin's value
        """
        return self.hal_comp[self.pin_name]

    @property
    def compname(self):
        '''The HAL component name; read-only property

        :returns: :py:class:`str` of component name
        '''
        return self.hal_comp.getprefix()

    @classmethod
    def _isclose(cls, a, b, rel_tol=1e-9, abs_tol=1e-9):
        return isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)


@attr.s
class RosHalPinPublisher(RosHalPin):
    '''HAL pin with attached ROS publisher

    This HAL pin is set with publishes its value on a ROS topic,
    :code:`<compname>/<name>` by default.  Its :py:func:`update` function
    should be called from the
    :py:func:`hal_hw_interface.ros_hal_component.RosHalComponent.update`
    function.

    It will only publish its value when the value has changed.  For
    :py:class:`FLOAT` types, absolute and relative tolerances are read
    from the :code:`<compname>/absolute_tolerance` and
    :code:`<compname>/relative_tolerance` ROS parameters and default
    to :code:`1e-9` each, respectively.

    The publisher message type will be one of :code:`[Bool, UInt32,
    Int32, Float64]` from the :py:mod:`std_msgs` ROS package, as appropriate
    for the HAL pin type.

    :param name: The HAL pin name
    :type name: str
    :param hal_type: HAL pin data type, one of :code:`['BIT', 'U32',
      'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of :code:`['IN', 'OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    :param pub_topic: ROS publisher topic
    :type pub_topic: str

    .. todo::  Parameter :code:`hal_dir` needs a validator; not all pin
      directions make sense for all subclasses

    .. todo::  Link documentation to ROS ``srv`` messages
    '''

    _default_hal_dir = 'IN'
    pub_topic = attr.ib()
    msg_type = attr.ib()
    last_value = attr.ib(default=None)

    # Attribute default factories
    @pub_topic.default
    def _pub_topic_default(self):
        return '{}/{}'.format(self.compname, self.pin_name)

    @msg_type.default
    def _msg_type_default(self):
        return {
            HalPinType('BIT'): Bool,
            HalPinType('U32'): UInt32,
            HalPinType('S32'): Int32,
            HalPinType('FLOAT'): Float64,
        }[self.hal_type]

    def _ros_init(self):
        self._ros_publisher_init()

    def _ros_publisher_init(self):
        rospy.loginfo('Creating publisher on topic "{}"'.format(self.pub_topic))
        self.pub = rospy.Publisher(
            self.pub_topic, self.msg_type, queue_size=1, latch=True
        )

    def _value_changed(self, value):
        if self.hal_type == HalPinType('FLOAT'):
            changed = self.last_value is None or not self._isclose(
                self.last_value,
                value,
                rel_tol=self.get_ros_param('relative_tolerance', 1e-9),
                abs_tol=self.get_ros_param('absolute_tolerance', 1e-9),
            )
        else:
            changed = self.last_value != value
        return changed

    def update(self):
        """If pin value has changed, publish to ROS topic
        """
        # rospy.logdebug(
        #     "publish_pins:  Publishing pin '%s' value '%s'" %
        #     (self.pin_name, self.get_pin()))
        value = self.get_pin()
        if self._value_changed(value):
            self.last_value = value
            self.pub.publish(value)


@attr.s
class RosHalPinSubscriber(RosHalPinPublisher):
    '''HAL pin with attached ROS publisher and subscriber

    This HAL pin isn't set via :py:func:`set_pin`, but subscribes to a
    ROS topic for its value.  As a subclass of
    :py:class:`RosHalPinPublisher`, it also publishes its value on a
    ROS topic from the :py:func:`update` function.

    The subscriber message type will be one of :code:`[Bool, UInt32,
    Int32, Float64]` from the :py:mod:`std_msgs` ROS package, as
    appropriate for the HAL pin type.

    :param name: The HAL pin name
    :type name: str
    :param hal_type: HAL pin data type, one of `['BIT', 'U32', 'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of `['OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    :param pub_topic: ROS publisher topic
    :type pub_topic: str
    :param sub_topic: ROS subscriber topic
    :type sub_topic: str
    '''

    _default_hal_dir = 'OUT'
    sub_topic = attr.ib()

    # Attribute default factories
    @sub_topic.default
    def _sub_topic_default(self):
        return '{}/{}'.format(self.compname, self.pin_name)

    def _ros_init(self):
        self._ros_publisher_init()
        self._ros_subscriber_init()

    def _ros_subscriber_init(self):
        rospy.loginfo(
            'Creating subscriber on topic "{}"'.format(self.sub_topic)
        )
        self.sub = rospy.Subscriber(
            self.sub_topic, self.msg_type, self._subscriber_cb
        )

    def _subscriber_cb(self, msg):
        # Sanity check
        if type(msg) is not self.msg_type:
            raise HalHWInterfaceException(
                "subscriber_cb:  Received incorrect message type '%s' for "
                "pin '%s' of msg type '%s'"
                % (type(msg), self.pin_name, self.msg_type)
            )

        self.set_pin(msg.data)


@attr.s
class RosHalPinService(RosHalPinPublisher):
    '''HAL pin with attached ROS service and publisher

    This HAL pin may be set via a ROS service, in addition to
    :py:func:`set_pin`.  As a subclass of
    :py:class:`RosHalPinPublisher`, it also publishes its value on a
    ROS topic from the :py:func:`update` function.

    The service message type will be :code:`SetBool` from the
    :py:mod:`std_srvs` package, or one of :code:`[SetUint32, SetInt32,
    SetFloat64]` from this package, as appropriate for the HAL pin
    type.  The default service name is :code:`<compname>/<name>`.

    :param name: The HAL pin name
    :type name: str
    :param hal_type: HAL pin data type, one of :code:`['BIT', 'U32',
      'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of :code:`['IN', 'OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    :param pub_topic: ROS publisher topic
    :type pub_topic: str
    :param service_name: ROS service name
    :type service_name: str
    '''

    _default_hal_dir = 'OUT'

    service_name = attr.ib()
    service_msg_type = attr.ib()

    # Attribute default factories
    @service_name.default
    def _service_name_default(self):
        return '{}/{}'.format(self.compname, self.pin_name)

    @service_msg_type.default
    def _service_msg_type_default(self):
        return {
            HalPinType('BIT'): SetBool,
            HalPinType('U32'): SetUInt32,
            HalPinType('S32'): SetInt32,
            HalPinType('FLOAT'): SetFloat64,
        }[self.hal_type]

    def _ros_init(self):
        self._ros_service_init()
        # Publish the value on a topic, too
        self._ros_publisher_init()

    def _ros_service_init(self):
        self.service = rospy.Service(
            self.service_name, self.service_msg_type, self._svc_cb
        )
        rospy.loginfo("Service {} created".format(self.service.resolved_name))

    def _svc_cb(self, req):
        self.set_pin(req.data)
        return True, 'OK'
