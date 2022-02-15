import pytest
from pprint import pformat

from ..ros_hal_pin import (
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
from ..ros_hal_component import RosHalComponent


class TestRosHalPin:
    test_class = RosHalPin
    comp_name = "mock_hal_comp"  # conftest.py
    rclpy_patches = [
        "hal_hw_interface.hal_obj_base.rclpy",
        "hal_hw_interface.ros_hal_component.rclpy",
    ]

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
        dict(  # 7 Test FLOAT IO
            name="float_io", hal_type="FLOAT", hal_dir="IO"
        ),
    ]

    case_defaults = dict(hal_type=None, hal_dir=HalPinDir("IN"))

    @pytest.fixture(params=obj_cases)
    def obj(self, request, mock_hal_comp, mock_rclpy):
        self.setup_hal_obj_base()

        # Munge test case data non-destructively
        case = self.case = self.case_defaults.copy()
        case.update(request.param)
        pin_name = self.pin_name = case.pop("name")
        for key in list(case.keys()):  # Remove inapplicable keys
            if key not in self.case_defaults:
                case.pop(key)

        obj = self.obj = self.test_class(pin_name, **case)
        print(f"test case '{pin_name}':", pformat(case))
        return obj

    # Message and pin data cases
    # Values are (bit, u32, s32, float)
    test_pvals_cases = [
        dict(  # 0 Test random
            val=(False, 23, -4, 1e-9),
            val2=(True, 27, 49, 3.98),
            changed=True,
        ),
        dict(  # 1 Test random
            val=(True, 0, -39958, 54.7),
            val2=(False, 199, -33399, 149.33285),
            changed=True,
        ),
        dict(  # 2 Test zeros -> random
            val=(False, 0, 0, 0.0),
            val2=(False, 0, 0, 0.0),
            changed=False,
        ),
        dict(  # 3 Test data unchanged
            val=(True, 199, -33399, 149.33285),
            val2=(True, 199, -33399, 149.33285),
            changed=False,
        ),
    ]

    test_pvals_indices = dict(BIT=0, U32=1, S32=2, FLOAT=3)

    @pytest.fixture(params=test_pvals_cases)
    def pvals(self, request):
        # Munge test_pvals_cases data, selecting value with relevant type
        # from tuples
        test_pvals = self.test_pvals = request.param.copy()
        index = self.test_pvals_indices[self.case["hal_type"]]
        for key, val in list(test_pvals.items()):
            if isinstance(val, tuple):
                test_pvals[key] = val[index]
        print("test_pvals:", pformat(test_pvals))
        return test_pvals

    def setup_hal_obj_base(self):
        # Remove stale HAL comp and init new one
        self.cleanup_fixture()

        class BogusComp(RosHalComponent):
            compname = self.comp_name

            def setup_component(self):
                pass

            def update(self):
                pass

        BogusComp(list())

    def cleanup_fixture(self):
        self.test_class._cached_objs.clear()

    #
    # Helpers
    #
    hal_to_msg_type_map = dict(BIT=Bool, FLOAT=Float64, U32=UInt32, S32=Int32)

    @property
    def msg_type(self):
        return self.hal_to_msg_type_map[self.case["hal_type"]]

    #
    # Base class tests
    #
    def test_ros_hal_pin_comp_name(self, mock_hal_comp, mock_rclpy):
        self.setup_hal_obj_base()
        obj = self.test_class("test_pin", "BIT")
        mock_hal_comp.reset_mock()
        obj.compname
        print(mock_hal_comp.mock_calls)
        assert mock_hal_comp.getprefix.call_count == 1

    def test_ros_hal_pin_obj_fixture(self, obj, mock_hal_comp, mock_rclpy):
        assert hasattr(self, "case")
        assert obj.name == self.pin_name
        assert obj.hal_type == HalPinType(self.case.get("hal_type"))
        assert obj.hal_dir == HalPinDir(self.case.get("hal_dir"))
        if "sub_topic" in self.case and hasattr(obj, "sub_topic"):
            assert obj.sub_topic == self.case.get("sub_topic")
        if "pub_topic" in self.case and hasattr(obj, "pub_topic"):
            assert obj.pub_topic == self.case.get("pub_topic")
        if "service_name" in self.case and hasattr(obj, "service_name"):
            assert obj.service_name == self.case.get("service_name")

    hal_to_python_type_map = dict(BIT=bool, FLOAT=float, U32=int, S32=int)

    def test_ros_hal_pin_data_fixture(self, obj, pvals):
        # obj fixture needed for hal_type cases
        python_type = self.hal_to_python_type_map[self.case["hal_type"]]
        for key in ("val2", "val"):
            assert key in pvals
            assert isinstance(pvals[key], python_type)
        assert (pvals["val"] != pvals["val2"]) is pvals["changed"]

    def test_ros_hal_pin_isclose(self):
        isclose_cases = [
            (0.0, 1.0, False),
            (100, 10, False),
            (0.0, 0.0, True),
            (1e-9, 1.5e-9, True),
        ]

        for a, b, res in isclose_cases:
            assert self.test_class._isclose(a, b, 1e-9, 1e-9) is res

    def test_ros_hal_pin_attrs(self, obj, mock_hal_comp):
        # Test that attributes are set as expected, including defaults
        assert obj.pin_name == self.pin_name
        assert obj.hal_comp is mock_hal_comp
        assert type(obj.hal_type) is HalPinType
        assert obj.hal_type == HalPinType(self.case["hal_type"])
        assert type(obj.hal_dir) is HalPinDir
        assert obj.hal_dir == HalPinDir(self.case["hal_dir"])

    def test_ros_hal_pin_pin_name(self, obj):
        # Test pin_name generation
        assert obj.pin_name == self.pin_name

    newpin_calls = 1

    def test_ros_hal_pin_newpin(self, obj, mock_hal_comp):
        # Test that hal.component.newpin() is called
        print(mock_hal_comp.newpin.mock_calls)
        mock_hal_comp.newpin.assert_any_call(
            self.pin_name,
            HalPinType(self.case["hal_type"]),
            HalPinDir(self.case["hal_dir"]),
        )
        assert mock_hal_comp.newpin.call_count == self.newpin_calls


class TestRosHalPinPublisher(TestRosHalPin):
    test_class = RosHalPinPublisher

    case_defaults = dict(
        hal_type=None, hal_dir=HalPinDir("IN"), pub_topic="hw/my_topic"
    )

    #
    # Helpers
    #
    def print_debug_info(self, extra_name=None, extra_val=None):
        print("self.obj._msg.data", self.obj._msg.data)
        print("self.pvals[self.pin_name]", self.pvals[self.pin_name])
        print("self.test_pvals['changed']", self.test_pvals["changed"])
        if extra_name is not None:
            print(extra_name, extra_val)

    #
    # Tests
    #
    def test_ros_hal_pin_attrs(self, obj, mock_hal_comp):
        super().test_ros_hal_pin_attrs(obj, mock_hal_comp)
        assert obj.pub_topic == self.case["pub_topic"]

    def test_ros_publisher_init(self, obj):
        assert obj.pub_topic == self.case["pub_topic"]
        print(self.node.create_publisher.mock_calls)
        self.node.create_publisher.assert_called_with(
            self.msg_type, self.case["pub_topic"], 1
        )
        assert obj.pub is self.publishers[self.case["pub_topic"]]

    def test_update(self, obj, pvals):
        assert self.rosparams.get("relative_tolerance", 1e-9) == 1e-9
        assert self.rosparams.get("absolute_tolerance", 1e-9) == 1e-9
        cur_value, last_value = (pvals["val"], pvals["val2"])

        # Set up publisher msg data != HAL pin value
        obj._msg.data = last_value
        self.pvals[self.pin_name] = cur_value

        # Test _value_changed()
        print("Before update():")
        self.print_debug_info()
        assert obj._value_changed(last_value) is pvals["changed"]

        # update() updates topic value to pin value
        obj.update()
        print("\nAfter update():")
        self.print_debug_info()
        assert obj._msg.data == cur_value
        assert obj._value_changed(cur_value) is False

        # Check calls
        print(f"pub.publish calls:  {obj.pub.publish.mock_calls}")
        obj.pub.publish.assert_called_with(obj._msg)


class TestRosHalPinSubscriber(TestRosHalPinPublisher):
    test_class = RosHalPinSubscriber

    case_defaults = dict(
        hal_type=None,
        hal_dir=HalPinDir("OUT"),
        pub_topic="hw/pub_topic",
        sub_topic="hw/sub_topic",
    )

    #
    # Tests
    #
    def test_ros_hal_pin_attrs(self, obj, mock_hal_comp):
        super().test_ros_hal_pin_attrs(obj, mock_hal_comp)
        assert obj.sub_topic == self.case["sub_topic"]

    def test_ros_subscriber_init(self, obj):
        assert obj.sub_topic == self.case["sub_topic"]
        print(self.node.create_subscription.mock_calls)
        self.node.create_subscription.assert_called_with(
            self.msg_type, self.case["sub_topic"], obj._subscriber_cb
        )
        assert obj.sub is self.subscriptions[self.case["sub_topic"]]

    def test_ros_hal_pin_subscriber_cb(self, obj, pvals):
        # Test that HAL pin is set in subscriber callback
        cur_value, new_value = (pvals["val"], pvals["val2"])

        self.pvals[self.pin_name] = obj._msg.data = cur_value
        msg = self.msg_type()
        msg.data = new_value
        self.print_debug_info("msg.data", msg.data)

        obj._subscriber_cb(msg)
        self.hal_comp.__setitem__.assert_called_with(self.pin_name, new_value)

        # Test that wrong msg type raises exception
        with pytest.raises(HalHWInterfaceException):
            obj._subscriber_cb(None)


class TestRosHalPinService(TestRosHalPinPublisher):
    test_class = RosHalPinService

    case_defaults = dict(
        hal_type=None,
        hal_dir=HalPinDir("OUT"),
        pub_topic="hw/my_topic",
        service_name="hw/my_srv",
    )

    #
    # Helpers
    #
    hal_to_srv_type_map = dict(
        BIT=SetBool, FLOAT=SetFloat64, U32=SetUInt32, S32=SetInt32
    )

    @property
    def service_msg_type(self):
        return self.hal_to_srv_type_map[self.case["hal_type"]]

    #
    # Tests
    #
    def test_ros_hal_pin_attrs(self, obj, mock_hal_comp):
        super().test_ros_hal_pin_attrs(obj, mock_hal_comp)
        assert obj.service_name == self.case["service_name"]
        assert obj.service_msg_type == self.service_msg_type

    def test_ros_service_init(self, obj):
        assert obj.service_name == self.case["service_name"]
        print(self.node.create_service.mock_calls)
        self.node.create_service.assert_called_with(
            self.service_msg_type, self.case["service_name"], obj._service_cb
        )
        assert obj.service is self.services[self.case["service_name"]]

    def test_service_cb(self, obj, pvals):
        # Test that the HAL pin is set during callback
        cur_value, call_value = (pvals["val"], pvals["val2"])
        self.pvals[self.pin_name] = obj._msg.data = cur_value
        msg_req = self.service_msg_type.Request()
        msg_rsp = self.service_msg_type.Response()
        print(dir(msg_req))
        msg_req.data = call_value

        print("Before service_cb:")
        self.print_debug_info("msg_req.data", msg_req.data)
        assert obj._value_changed(call_value) is pvals["changed"]

        msg_rsp = obj._service_cb(msg_req, msg_rsp)

        print("After service_cb:")
        self.print_debug_info("msg_req.data", msg_req.data)
        assert obj._value_changed(call_value) is False

        obj.hal_comp.__setitem__.assert_called_with(self.pin_name, call_value)
        assert (msg_rsp.success, msg_rsp.message) == (True, "OK")
