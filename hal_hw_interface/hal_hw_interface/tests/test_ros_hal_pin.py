# -*- coding: utf-8 -*-
import pytest

from hal_hw_interface.ros_hal_pin import (
    HalObjBase,
    RosHalPin,
    RosHalPinSubscriber,
    RosHalPinPublisher,
    RosHalPinService,
    HalHWInterfaceException,
    HalPinDir,
    HalPinType,
    SetBool,
    SetUInt32,
    SetInt32,
    SetFloat64,
    Bool,
    Float64,
    UInt32,
    Int32,
)
from std_msgs.msg import UInt16  # For invalid test case


class TestRosHalPin(object):
    test_class = RosHalPin
    default_hal_type = None
    default_hal_dir = HalPinDir("IN")
    compname = "mock_hal_comp_obj"  # conftest.py
    extra_attrs = []

    #
    # Object and data fixtures
    #

    # List of object fixture parameter test cases to run for each class;
    # contains enough attributes for superset of all classes
    obj_cases = [
        dict(  # 0 Test BIT, IO, names
            name="reset",
            hal_type="BIT",
            hal_dir="IO",
            sub_topic="/robot/reset",
            pub_topic="/robot/reset",
            service_name="/robot/reset",
        ),
        dict(name="bool", hal_type="BIT", hal_dir="OUT"),  # 1 Test BIT OUT
        dict(name="bool", hal_type="BIT"),  # 2 Test BIT defaults
        dict(name="u32_out", hal_type="U32", hal_dir="OUT"),  # 3 Test U32 OUT
        dict(name="u32_out", hal_type="U32", hal_dir="IO"),  # 4 Test U32 IN
        dict(name="u32_out", hal_type="U32"),  # 5 Test U32 default
        dict(name="s32_out", hal_type="S32"),  # 6 Test S32 default
        dict(
            name="float_io", hal_type="FLOAT", hal_dir="IO"
        ),  # 7 Test FLOAT IO
    ]

    @pytest.fixture(params=obj_cases)
    def obj(self, request, mock_comp_obj, mock_rospy, mock_redis_client_obj):
        self.setup_hal_obj_base(mock_comp_obj)
        mock_comp_obj.setprefix(self.compname)
        attrs = dict()
        params = request.param.copy()
        params.setdefault("hal_dir", self.default_hal_dir)
        name = params.pop("name")
        attr_names = ["hal_comp", "hal_type", "hal_dir", "msg_type"]
        attr_names += self.extra_attrs
        for attr_name in attr_names:
            if attr_name in params:
                attrs[attr_name] = params.pop(attr_name)
        obj = self.test_class(name, **attrs)
        obj._p = request.param  # Send test params in
        return obj

    # Message and pin data cases
    # Values are (bit, u32, s32, float)
    data_cases = [
        dict(  # 0 Test random
            pin_value=(False, 23, -4, 1e-9), other_value=(True, 27, 49, 3.98)
        ),
        dict(  # 1 Test random
            pin_value=(True, 0, -39958, 54.7),
            other_value=(False, 199, -33399, 149.33285),
        ),
        dict(  # 2 Test zeros -> random
            pin_value=(False, 0, 0, 0.0),
            other_value=(False, 0, 0, 0.0),
            changed=False,
        ),
        dict(  # 3 Test data unchanged
            pin_value=(True, 199, -33399, 149.33285),
            other_value=(True, 199, -33399, 149.33285),
            changed=False,
        ),
    ]

    @pytest.fixture(params=data_cases)
    def data(self, request):
        request.param.setdefault("changed", True)
        return request.param

    isclose_cases = [
        (0.0, 1.0, False),
        (100, 10, False),
        (0.0, 0.0, True),
        (1e-9, 1.5e-9, True),
    ]

    @pytest.fixture(params=isclose_cases)
    def isclose_case(self, request):
        return request.param

    def setup_hal_obj_base(self, mock_comp_obj):
        # Remove stale HAL comp and init new one
        self.cleanup_fixture(HalObjBase)

        class BogusComp(HalObjBase):
            compname = self.compname

        mock_comp_obj.setprefix(self.compname)

        BogusComp().init_hal_comp()

    def cleanup_fixture(self, test_class=None):
        if test_class is None:
            test_class = self.test_class
        for key in list(test_class._cached_objs.keys()):
            test_class._cached_objs.pop(key)

    #
    # Helpers
    #
    def get_obj_test_param(self, obj, param, default=None):
        return obj._p.get(param, default)

    def obj_test_name(self, obj):
        return self.get_obj_test_param(obj, "name")

    def hal_dir(self, obj):
        return self.get_obj_test_param(obj, "hal_dir", self.default_hal_dir)

    def hal_type(self, obj):
        return self.get_obj_test_param(obj, "hal_type")

    def msg_type(self, obj):
        return dict(BIT=Bool, FLOAT=Float64, U32=UInt32, S32=Int32)[
            self.hal_type(obj)
        ]

    data_indexes = {
        HalPinType("BIT"): 0,
        HalPinType("U32"): 1,
        HalPinType("S32"): 2,
        HalPinType("FLOAT"): 3,
    }

    def set_last_value(self, obj, param):
        index = self.data_indexes.get(obj.hal_type)
        value = param["other_value"][index]
        obj.last_value = value
        return value

    def set_pin(self, obj, param):
        if isinstance(param, dict):
            index = self.data_indexes.get(obj.hal_type)
            value = param["pin_value"][index]
        else:
            value = param
        obj.hal_comp.set_pin(self.obj_test_name(obj), value)
        return value

    def other_value(self, obj, param):
        index = self.data_indexes.get(obj.hal_type)
        return param["other_value"][index]

    def ros_name(self, obj):
        return "{}/{}".format(self.compname, self.obj_test_name(obj))

    #
    # Base class tests
    #
    def test_ros_hal_pin_compname(self, mock_comp_obj, all_patches):
        self.setup_hal_obj_base(mock_comp_obj)
        obj = self.test_class("test_pin", "BIT")
        mock_comp_obj.reset_mock()
        obj.compname
        print(mock_comp_obj.mock_calls)
        assert mock_comp_obj.getprefix.call_count == 1

    def test_ros_hal_pin_obj_fixture(
        self, obj, mock_comp_obj, mock_rospy, mock_objs
    ):
        assert hasattr(obj, "_p")
        params = obj._p
        assert obj.name == params["name"]
        assert obj.hal_type == HalPinType(
            params.get("hal_type", self.default_hal_type)
        )
        assert obj.hal_dir == HalPinDir(
            params.get("hal_dir", self.default_hal_dir)
        )
        if "sub_topic" in params and hasattr(obj, "sub_topic"):
            assert obj.sub_topic == params.get("sub_topic")
        if "pub_topic" in params and hasattr(obj, "pub_topic"):
            assert obj.pub_topic == params.get("pub_topic")
        if "service_name" in params and hasattr(obj, "service_name"):
            assert obj.pub_topic == params.get("service_name")

    def test_ros_hal_pin_data_fixture(self, data):
        for key in ("other_value", "pin_value"):
            assert key in data
            assert len(data[key]) == 4
            assert isinstance(data[key][0], bool)
            assert isinstance(data[key][1], int)
            assert data[key][1] >= 0
            assert isinstance(data[key][2], int)
            assert isinstance(data[key][3], float)

    def test_ros_hal_pin_isclose_case_fixture(self, isclose_case):
        assert len(isclose_case) == 3
        assert isinstance(isclose_case[2], bool)

    def test_ros_hal_pin_isclose(self, isclose_case):
        a, b, res = isclose_case
        assert self.test_class._isclose(a, b, 1e-9, 1e-9) is res

    def test_ros_hal_pin_attrs(self, obj, mock_comp_obj):
        """Test that attributes are set as expected, including defaults"""
        assert obj.pin_name == self.obj_test_name(obj)
        assert obj.hal_comp is mock_comp_obj
        assert type(obj.hal_type) is HalPinType
        assert obj.hal_type == HalPinType(self.hal_type(obj))
        assert type(obj.hal_dir) is HalPinDir
        assert obj.hal_dir == HalPinDir(self.hal_dir(obj))

    def test_ros_hal_pin_pin_name(self, obj):
        """Test pin_name generation"""
        assert obj.pin_name == self.obj_test_name(obj)

    newpin_calls = 1

    def test_ros_hal_pin_newpin(self, obj, mock_comp_obj):
        """Test that hal.component.newpin() is called"""
        print(mock_comp_obj.newpin.mock_calls)
        mock_comp_obj.newpin.assert_any_call(
            self.obj_test_name(obj),
            HalPinType(self.hal_type(obj)),
            HalPinDir(self.hal_dir(obj)),
        )
        assert mock_comp_obj.newpin.call_count == self.newpin_calls

    def test_ros_hal_pin_default_attr_hal_type(self, all_patches):
        """Test default hal_type"""
        # This one creates the object in the test, so `all_patches`
        # needed
        if self.default_hal_type is None:
            with pytest.raises(TypeError):
                self.test_class("default_hal_type")
        else:
            obj = self.test_class("default_hal_type")
            assert obj.hal_type == self.default_hal_type


class TestRosHalPinPublisher(TestRosHalPin):
    default_hal_dir = HalPinDir("IN")
    test_class = RosHalPinPublisher
    extra_attrs = ["pub_topic"]

    #
    # Helpers
    #
    def pub_topic(self, obj):
        return self.get_obj_test_param(obj, "pub_topic", self.ros_name(obj))

    def set_last_value(self, obj, param):
        # Set msg.data
        value = super().set_last_value(obj, param)
        obj._msg.data = value
        return value

    #
    # Tests
    #
    def test_ros_hal_pin_attrs(self, obj, mock_comp_obj):
        super(TestRosHalPinPublisher, self).test_ros_hal_pin_attrs(
            obj, mock_comp_obj
        )
        assert obj.pub_topic == self.pub_topic(obj)

    def test_ros_hal_pin_publisher_init(self, obj, mock_objs):
        """Test that __init__() creates rospy.Publisher object"""
        print(mock_objs["rospy_Publisher"].mock_calls)
        assert obj.pub_topic == self.pub_topic(obj)
        mock_objs["rospy_Publisher"].assert_called_with(
            self.pub_topic(obj), self.msg_type(obj), queue_size=1, latch=True
        )
        assert obj.pub is mock_objs["rospy_Publisher_obj"]

    def test_ros_hal_pin_publisher_value_changed(self, obj, data, mock_objs):
        """Test _value_changed() function"""
        assert obj.get_ros_param("relative_tolerance", 1e-9) == 1e-9
        assert obj.get_ros_param("absolute_tolerance", 1e-9) == 1e-9

        last_value = self.set_last_value(obj, data)
        cur_value = self.set_pin(obj, data)
        same = last_value == cur_value
        assert same is not data.get("changed")
        assert obj._value_changed(cur_value) is data.get("changed")

    def test_ros_hal_pin_publisher_update(self, obj, data):
        self.set_last_value(obj, data)
        cur_value = self.set_pin(obj, data)
        obj.update()
        assert obj._msg.data == cur_value
        print(f"pub.publish calls:  {obj.pub.publish.mock_calls}")
        if data.get("changed"):
            obj.pub.publish.assert_called_with(obj._msg)
        else:
            obj.pub.publish.assert_not_called


class TestRosHalPinSubscriber(TestRosHalPinPublisher):
    default_hal_dir = HalPinDir("OUT")
    test_class = RosHalPinSubscriber
    extra_attrs = ["pub_topic", "sub_topic"]

    #
    # Helpers
    #
    def sub_topic(self, obj):
        return self.get_obj_test_param(obj, "sub_topic", self.ros_name(obj))

    #
    # Tests
    #
    def test_ros_hal_pin_attrs(self, obj, mock_comp_obj):
        super(TestRosHalPinSubscriber, self).test_ros_hal_pin_attrs(
            obj, mock_comp_obj
        )
        assert obj.sub_topic == self.sub_topic(obj)

    def test_ros_hal_pin_subscriber_init(self, obj, mock_objs):
        """Test that __init__() creates rospy.Subscriber object"""
        mock_objs["rospy_Subscriber"].assert_called_with(
            self.sub_topic(obj), self.msg_type(obj), obj._subscriber_cb
        )
        assert obj.sub is mock_objs["rospy_Subscriber_obj"]

    def test_ros_hal_pin_subscriber_cb(self, obj, data, mock_comp_obj):
        """Test that HAL pin is set in subscriber callback"""
        other_value = self.other_value(obj, data)
        obj._subscriber_cb(self.msg_type(obj)(other_value))
        mock_comp_obj.__setitem__.assert_called_with(
            self.obj_test_name(obj), other_value
        )

    def test_ros_hal_pin_subscriber_bad_msg_type(self, obj):
        """Test that invalid message types are caught"""
        msg = UInt16(42)
        with pytest.raises(HalHWInterfaceException):
            obj._subscriber_cb(msg)


class TestRosHalPinService(TestRosHalPinPublisher):
    default_hal_dir = HalPinDir("OUT")
    test_class = RosHalPinService
    extra_attrs = ["pub_topic", "service_name"]

    #
    # Helpers
    #
    def service_name(self, obj):
        return self.get_obj_test_param(obj, "service_name", self.ros_name(obj))

    def service_msg_type(self, obj):
        return dict(BIT=SetBool, FLOAT=SetFloat64, U32=SetUInt32, S32=SetInt32)[
            self.hal_type(obj)
        ]

    #
    # Tests
    #
    def test_ros_hal_pin_attrs(self, obj, mock_comp_obj):
        super(TestRosHalPinService, self).test_ros_hal_pin_attrs(
            obj, mock_comp_obj
        )
        assert obj.service_name == self.service_name(obj)
        assert obj.service_msg_type == self.service_msg_type(obj)

    def test_ros_hal_pin_service_init(self, obj, mock_objs):
        """Test that __init__() creates rospy.Service object"""
        assert obj.service_name == self.service_name(obj)
        mock_objs["rospy_Service"].assert_called_with(
            self.service_name(obj), self.service_msg_type(obj), obj._svc_cb
        )
        assert obj.service is mock_objs["rospy_Service_obj"]

    def test_ros_hal_pin_service_cb(self, obj, data):
        """Test that the HAL pin is set during callback"""
        msg = self.service_msg_type(obj)
        call_value = self.other_value(obj, data)
        msg.data = call_value
        reply = obj._svc_cb(msg)
        obj.hal_comp.__setitem__.assert_called_with(
            self.obj_test_name(obj), call_value
        )
        assert reply == (True, "OK")
