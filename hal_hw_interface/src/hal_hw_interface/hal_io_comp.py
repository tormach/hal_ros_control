from hal_hw_interface.ros_hal_component import RosHalComponent
from hal_hw_interface.ros_hal_pin import (
    RosHalPinSubscriber,
    RosHalPinPublisher,
    RosHalPinService,
)
from hal_hw_interface.redis_store_hal_pin import RedisStoreHalPin


class HalIO(RosHalComponent):
    """HAL user component that sets up HAL pins connected to ROS

    This component queries the ROS parameter server for
    :code:`hal_io/subscribe_pins`, :code:`hal_io/publish_pins` and
    :code:`hal_io/service_pins` parameters, lists of pin names to pin
    data mappings.

    The pin data must be a dictionary of pin configuration.

    Example:

    .. code-block:: yaml

        hal_io:
          subscribe_pins:
            enable:
              hal_type: BIT
              hal_dir: OUT
          publish_pins:
            digital_in_1:
              hal_type: BIT
              hal_dir: IN
          service_pins:
            encoder_scale:
              hal_type: FLOAT
              hal_dir: OUT
          redis_pins:
            current_tool:
              hal_type: U32
              hal_dir: IO

    The periodic :py:func:`update` function calls the pins'
    :py:func:`update` functions, if any.
    """

    compname = 'hal_io'

    def setup_component(self):
        """Load pin configuration from ROS param server and create pin objects"""
        self.pins = []
        self.reset_pin = None
        pin_class_map = dict(
            subscribe_pins=RosHalPinSubscriber,
            publish_pins=RosHalPinPublisher,
            service_pins=RosHalPinService,
            redis_pins=RedisStoreHalPin,
        )
        for config_key, pin_class in pin_class_map.items():
            pins = self.get_ros_param(config_key, dict())
            for pin_name, pin_data in pins.items():
                p = pin_class(pin_name, **pin_data)
                self.pins.append(p)
        if self.get_ros_param('resetable', False):
            self.reset_pin = RosHalPinPublisher(
                'reset', hal_type='BIT', hal_dir='IO'
            )

    def update(self):
        """Run pin `update()` functions"""
        reset = False
        if self.reset_pin is not None:
            self.reset_pin.update()
            reset = self.reset_pin.get_pin()
        for p in self.pins:
            p.update(reset)
        if reset:
            self.reset_pin.set_pin(False)  # signal reset completed
