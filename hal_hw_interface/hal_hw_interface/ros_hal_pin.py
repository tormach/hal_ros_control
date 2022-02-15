"""
:synopsis: ROS-connected HAL pin objects.

.. moduleauthor:: John Morris <john@dovetail-automata.com>

.. inheritance-diagram::
    hal_hw_interface.ros_hal_pin.RosHalPin
    hal_hw_interface.ros_hal_pin.RosHalPinPublisher
    hal_hw_interface.ros_hal_pin.RosHalPinSubscriber
    hal_hw_interface.ros_hal_pin.RosHalPinService
"""

import attr
from .hal_obj_base import HalObjBase
from .exception import HalHWInterfaceException
from .hal_pin_attrs import HalPinDir, HalPinType
from std_msgs.msg import Bool, UInt32, Int32, Float64
from std_srvs.srv import SetBool
from hal_hw_interface_msgs.srv import SetUInt32, SetInt32, SetFloat64
from math import isclose


@attr.s
class RosHalPin(HalObjBase):
    """
    Basic HAL pin for use in user components.

    Intended for HAL "user" components built with the
    :py:class:`hal_hw_interface.ros_hal_component.RosHalComponent`
    class.

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
    """

    _default_hal_type = None
    _default_hal_dir = "IN"  # Subclasses may override for hal_dir attribute

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
            raise TypeError("%s requires hal_type= argument" % self.__class__)
        return HalPinType(self._default_hal_type)

    @property
    def pin_name(self):
        """
        Accessor for the read-only pin_name.

        In some subclasses, this may not be the same as the
        :code:`name` parameter supplied to the constructor.

        :returns:  :py:class:`str` pin name
        """
        return self.name

    def __attrs_post_init__(self):
        self._hal_init()
        self._ros_init()

    def _hal_init(self):
        self.hal_pin = self.hal_comp.newpin(
            self.pin_name, self.hal_type, self.hal_dir
        )

        self.logger.info(
            'Created pin "{}" type/dir "{}/{}"'.format(
                self.pin_name, self.hal_type, self.hal_dir
            )
        )

    def _ros_init(self):
        # May be implemented in subclasses
        pass

    def update(self):
        """
        Update the HAL pin object.

        Overridden by some subclasses.
        """

    def set_pin(self, value):
        """
        Set the HAL pin's value.

        Does not apply to pins initialized with :code:`hal_dir='IN'`.

        :param value: The value to set
        """
        self.hal_comp[self.pin_name] = value

    def get_pin(self):
        """
        Get the HAL pin's value.

        :returns: HAL pin's value
        """
        return self.hal_comp[self.pin_name]

    @property
    def compname(self):
        """
        Accessor for the read-only HAL component name.

        :returns: :py:class:`str` of component name
        """
        return self.hal_comp.getprefix()

    @classmethod
    def _isclose(cls, a, b, rel_tol=1e-9, abs_tol=1e-9):
        return isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)


@attr.s
class RosHalPinPublisher(RosHalPin):
    """
    HAL pin with attached ROS publisher.

    This HAL pin is set with publishes its value on a ROS topic,
    :code:`<compname>/<name>` by default.  Its :py:func:`update`
    function should be called from the
    :py:func:`hal_hw_interface.ros_hal_component.RosHalComponent.update`
    function.

    Value changes are logged.  For :py:class:`FLOAT` types, absolute
    and relative tolerances are read from the
    :code:`<compname>/absolute_tolerance` and
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
    """

    _default_hal_dir = "IN"
    pub_topic = attr.ib()
    msg_type = attr.ib()

    # Attribute default factories
    @pub_topic.default
    def _pub_topic_default(self):
        return self.pin_name

    _pin_to_msg_type_map = {
        HalPinType("BIT"): Bool,
        HalPinType("U32"): UInt32,
        HalPinType("S32"): Int32,
        HalPinType("FLOAT"): Float64,
    }

    _pin_to_python_type_map = {
        HalPinType("BIT"): bool,
        HalPinType("U32"): int,
        HalPinType("S32"): int,
        HalPinType("FLOAT"): float,
    }

    @msg_type.default
    def _msg_type_default(self):
        return self._pin_to_msg_type_map[self.hal_type]

    def _ros_init(self):
        self._ros_publisher_init()
        msg_type = self._pin_to_msg_type_map[self.hal_type]
        self._msg = msg_type()
        self.update()

    def _ros_publisher_init(self):
        self.logger.info(f'Creating publisher on topic "{self.pub_topic}"')
        self.pub = self.node.create_publisher(self.msg_type, self.pub_topic, 1)

    def _value_changed(self, value):
        if self.hal_type == HalPinType("FLOAT"):
            return not self._isclose(
                self.get_pin(),
                value,
                rel_tol=self.get_ros_param("relative_tolerance", 1e-9),
                abs_tol=self.get_ros_param("absolute_tolerance", 1e-9),
            )
        else:
            return self.get_pin() != value

    def update(self):
        """Publish pin value to ROS topic."""
        if self._value_changed(self._msg.data):
            self.logger.info(
                f"Pin {self.pin_name} changed:"
                f"  old={self._msg.data}; new={self.get_pin()}"
            )
        value = self._pin_to_python_type_map[self.hal_type](self.get_pin())
        self._msg.data = value
        self.pub.publish(self._msg)


@attr.s
class RosHalPinSubscriber(RosHalPinPublisher):
    """
    HAL pin with attached ROS publisher and subscriber.

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
    """

    _default_hal_dir = "OUT"
    sub_topic = attr.ib()

    # Attribute default factories
    @sub_topic.default
    def _sub_topic_default(self):
        return "{}/{}".format(self.compname, self.pin_name)

    def _ros_init(self):
        super()._ros_init()
        self._ros_subscriber_init()

    def _ros_subscriber_init(self):
        self.logger.info(f'Creating subscriber on topic "{self.sub_topic}"')
        self.sub = self.node.create_subscription(
            self.msg_type, self.sub_topic, self._subscriber_cb
        )

    def _subscriber_cb(self, msg):
        # Sanity check
        if type(msg) is not self.msg_type:
            raise HalHWInterfaceException(
                f"subscriber_cb:  Received incorrect message type '{type(msg)}'"
                f"for pin '{self.pin_name}' of msg type '{self.msg_type}'"
            )

        self._msg.data = msg.data
        self.set_pin(msg.data)
        self.update()


@attr.s
class RosHalPinService(RosHalPinPublisher):
    """
    HAL pin with attached ROS service and publisher.

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
    """

    _default_hal_dir = "OUT"

    service_name = attr.ib()
    service_msg_type = attr.ib()

    # Attribute default factories
    @service_name.default
    def _service_name_default(self):
        return "{}/{}".format(self.compname, self.pin_name)

    _pin_to_service_msg_type_map = {
        HalPinType("BIT"): SetBool,
        HalPinType("U32"): SetUInt32,
        HalPinType("S32"): SetInt32,
        HalPinType("FLOAT"): SetFloat64,
    }

    @service_msg_type.default
    def _service_msg_type_default(self):
        return self._pin_to_service_msg_type_map[self.hal_type]

    def _ros_init(self):
        super()._ros_init()
        self._ros_service_init()

    def _ros_service_init(self):
        self.service = self.node.create_service(
            self.service_msg_type, self.service_name, self._service_cb
        )
        self.logger.info(f"Service {self.service_name} created")

    def _service_cb(self, req, rsp):
        self.set_pin(req.data)
        self.update()
        rsp.success, rsp.message = (True, "OK")
        return rsp
