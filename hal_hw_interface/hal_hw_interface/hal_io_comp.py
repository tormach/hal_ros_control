from hal_hw_interface.ros_hal_component import RosHalComponent
from hal_hw_interface.ros_hal_pin import (
    RosHalPinSubscriber,
    RosHalPinPublisher,
    RosHalPinService,
)


class HalIO(RosHalComponent):
    """
    HAL user component that sets up HAL pins connected to ROS.

    This component queries the ROS parameter server for
    :code:`hal_io/subscribe_pins`, :code:`hal_io/publish_pins` and
    :code:`hal_io/service_pins` parameters, lists of pin names to pin
    data mappings.

    The pin data must be a dictionary of pin configuration; example:

    .. code-block:: yaml

        hal_io:
          subscribe_pins:
            enable:
              hal_type: BIT
              hal_dir: OUT
          publish_pins:
            digital_out_1:
              hal_type: BIT
              hal_dir: IN
          service_pins:
            encoder_scale:
              hal_type: FLOAT
              hal_dir: OUT

    The periodic :py:func:`update` function calls the pins'
    :py:func:`update` functions, if any.
    """

    compname = "hal_io"

    pin_class_map = dict(
        subscribe_pins=RosHalPinSubscriber,
        publish_pins=RosHalPinPublisher,
        service_pins=RosHalPinService,
    )

    def setup_component(self):
        """Create pin objects from ROS param server configuration."""
        self.pins = []
        for config_key, pin_class in self.pin_class_map.items():
            pins = self.get_ros_param(config_key, dict())
            for pin_name, pin_data in pins.items():
                p = pin_class(pin_name, **pin_data)
                self.pins.append(p)

    def update(self):
        """Run pin :py:func:`update` functions."""
        for p in self.pins:
            p.update()
