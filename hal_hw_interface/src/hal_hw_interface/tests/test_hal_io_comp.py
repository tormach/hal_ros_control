# -*- coding: utf-8 -*-
import pytest
from hal_hw_interface.hal_io_comp import HalIO


class TestHalIO(object):
    test_class = HalIO

    @pytest.fixture
    def pin_params(self, mock_rospy, mock_objs):
        gp = mock_objs["rospy_get_param"]

        test_pins = dict(
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
        for key, val in test_pins.items():
            gp.set_key("hal_io/%s" % key, val)
        return gp, test_pins

    @pytest.fixture
    def obj(self, mock_comp_obj, mock_rospy, mock_objs, pin_params):
        for key in list(self.test_class._cached_objs.keys()):
            self.test_class._cached_objs.pop(key)

        mock_comp_obj.setprefix("hal_io")
        return self.test_class()

    def test_hal_io_comp_fixture(self, pin_params):
        get_param, test_pins = pin_params
        print(get_param("hal_io/publish_pins"))
        assert isinstance(get_param("hal_io/publish_pins"), dict)
        assert "bit_sub" in get_param("hal_io/subscribe_pins")

    def test_hal_io_comp_setup_component(self, obj, pin_params):
        get_param, test_pins = pin_params
        obj.setup_component()

        # Put pins in dict
        obj_pins = dict()
        for pin in obj.pins:
            obj_pins[pin.name] = pin

        for config_key, pins in test_pins.items():
            # Check param server was queried
            print("Pin class:  %s" % config_key)
            get_param.assert_any_call("hal_io/%s" % config_key, {})
            for pin_name, pin_data in pins.items():
                # Check pin was created
                assert pin_name in obj_pins
                # Check pin attributes
                pin = obj_pins[pin_name]
                print("  name %s; data %s" % (pin_name, pin_data))
                print("  pin %s" % pin)
                for key, val in pin_data.items():
                    assert str(getattr(pin, key)) == val

    def test_hal_io_comp_update(self, obj, pin_params, mock_comp_obj):
        # Check that pins' update() method was called
        get_param, test_pins = pin_params
        obj.setup_component()

        # Set pin values
        for pin in obj.pins:
            mock_comp_obj.set_pin(pin.name, 1)

        # Run update() and check that pins changed
        print("----------- Running obj.update()")
        obj.update()
        for pin in obj.pins:
            print(f"- pin {pin}")
            pin.pub.publish.assert_called()
