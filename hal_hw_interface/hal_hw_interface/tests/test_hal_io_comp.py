import pytest
from hal_hw_interface.hal_io_comp import HalIO
from pprint import pformat
import yaml
import os


class TestHalIO:
    test_class = HalIO
    rclpy_patches = [
        "hal_hw_interface.hal_obj_base.rclpy",
        "hal_hw_interface.ros_hal_component.rclpy",
    ]

    def flatten_keys(self, prefix, data, result=dict()):
        for key, val in data.items():
            new_prefix = "/".join([prefix, key])
            if not isinstance(val, dict):
                result[new_prefix] = val
            else:
                self.flatten_keys(new_prefix, val, result)
        return result

    @pytest.fixture
    def config(self):
        self.config_file = os.path.join(
            os.path.dirname(__file__), "hal_io.conf.yaml"
        )
        with open(self.config_file, "r") as f:
            self.config = yaml.safe_load(f)
        print("hal_io config:", pformat(self.config))
        yield self.config

    @pytest.fixture
    def obj(self, mock_hal_comp, mock_rclpy, config):
        self.test_class._cached_objs.clear()
        obj = self.test_class([self.config_file])
        obj.setup_component()
        yield obj

    def test_hal_io_comp_setup_component(self, obj, config):
        # Put pins in dict
        obj_pins = dict()
        for pin in obj.pins:
            obj_pins[pin.name] = pin

        for config_key, pins in config.items():
            print("\n\nPin class:  %s" % config_key)
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

    def test_hal_io_comp_update(self, obj, mock_hal_comp):
        # Check that pins' update() method was called
        for pin in obj.pins:  # Init pin values
            self.pvals[pin.name] = 1

        # Run update() and check that pins changed
        obj.update()
        print("publishers", pformat(self.publishers))
        print("pin values", pformat(self.pvals))
        for pin in obj.pins:
            print(f"- pin {pin}")
            publisher = self.publishers[pin.pub_topic]
            publisher.publish.assert_called()
