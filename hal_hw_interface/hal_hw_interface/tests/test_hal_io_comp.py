import pytest
from hal_hw_interface.hal_io_comp import HalIO
from pprint import pformat


class TestHalIO:
    test_class = HalIO

    def flatten_keys(self, prefix, data, result=dict()):
        for key, val in data.items():
            new_prefix = "/".join([prefix, key])
            if not isinstance(val, dict):
                result[new_prefix] = val
            else:
                self.flatten_keys(new_prefix, val, result)
        return result

    @pytest.fixture
    def pin_params(self, mock_rclpy):
        self.rosparams = test_pins = dict(
            publish_pins=dict(
                bit_pub=dict(hal_type="HAL_BIT"),
                u32_pub=dict(hal_type="HAL_U32"),
                s32_pub=dict(hal_type="HAL_S32", hal_dir="HAL_IO"),
                float_pub=dict(hal_type="HAL_FLOAT", pub_topic="/float/topic"),
            ),
            subscribe_pins=dict(
                bit_sub=dict(hal_type="HAL_BIT", sub_topic="/bit/topic"),
                u32_sub=dict(hal_type="HAL_U32", hal_dir="HAL_IO"),
                s32_sub=dict(hal_type="HAL_S32"),
                float_sub=dict(hal_type="HAL_FLOAT"),
            ),
            service_pins=dict(
                bit_svc=dict(hal_type="HAL_BIT"),
                u32_svc=dict(hal_type="HAL_U32", hal_dir="HAL_IO"),
                s32_svc=dict(hal_type="HAL_S32", service_name="/s32/svc"),
                float_svc=dict(hal_type="HAL_FLOAT"),
            ),
        )
        # self.rosparams = self.flatten_keys("hal_io", test_pins)
        print("rosparams", pformat(self.rosparams))
        yield test_pins

    @pytest.fixture
    def obj(self, mock_hal_comp, mock_rclpy, pin_params):
        self.test_class._cached_objs.clear()
        obj = self.test_class(list())
        obj.setup_component()
        yield obj

    def test_hal_io_comp_setup_component(self, obj, pin_params):
        test_pins = pin_params
        obj.setup_component()

        # Put pins in dict
        obj_pins = dict()
        for pin in obj.pins:
            obj_pins[pin.name] = pin

        for config_key, pins in test_pins.items():
            # Check param server was queried
            print("Pin class:  %s" % config_key)
            self.node.declare_parameter.assert_any_call(config_key, {})
            print("pins:", pformat(pins))
            print("obj_pins:", pformat(obj_pins))
            for pin_name, pin_data in pins.items():
                print("pin_name:", pin_name, "pin_data:", pin_data)
                # Check pin was created
                assert pin_name in obj_pins
                # Check pin attributes
                pin = obj_pins[pin_name]
                print("  name %s; data %s" % (pin_name, pin_data))
                print("  pin %s" % pin)
                for key, val in pin_data.items():
                    assert str(getattr(pin, key)) == val

    def test_hal_io_comp_update(self, obj, pin_params, mock_hal_comp):
        # Check that pins' update() method was called
        obj.setup_component()

        # Set pin values
        for pin in obj.pins:
            self.pvals[pin.name] = 1

        # Run update() and check that pins changed
        print("----------- Running obj.update()")
        obj.update()
        print("publishers", pformat(self.publishers))
        print("pins", pformat(self.pvals))
        for pin in obj.pins:
            print(f"- pin {pin}")
            publisher = self.publishers[pin.pub_topic]
            print(publisher.mock_calls)
            publisher.publish.assert_called()
