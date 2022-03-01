from launch.event import Event


class HalReady(Event):
    """Event emitted when parts of HAL bringup are complete."""

    name = "hal_hw_interface.launch.hal_ready_event"

    def __init__(self, name):
        """Create a HalReady event."""
        self.__name = name

    @property
    def name(self):
        """Getter for name."""
        return self.__name

    def __repr__(self):
        return f"<HalReady('{self.name}') event object>"
