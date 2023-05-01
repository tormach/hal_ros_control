import pytest
from hal_hw_interface.ros_hal_component import RosHalComponent


class TestRosHalComponent:
    # Simplest test case
    class StubComp(RosHalComponent):
        compname = "test_stub"

        def setup_component(self):
            self.initialized = True
            self.count = 0
            super().setup_component()

        def update(self):
            self.count += 1

    test_class = StubComp
    comp_name = test_class.compname
    rclpy_patches = [
        "hal_hw_interface.hal_obj_base.rclpy",
        "hal_hw_interface.ros_hal_component.rclpy",
    ]

    @pytest.fixture
    def obj(self, mock_hal_comp, mock_rclpy):
        self.test_class._cached_objs.clear()  # Clean fixture
        self.rosparams = dict(
            update_rate=20,
            relative_tolerance=1e-9,
            absolute_tolerance=1e-9,
        )
        obj = self.test_class(list())
        obj.setup_component()
        yield obj

    def test_init(self, obj):
        assert hasattr(obj, "node")  # Called init_ros_node
        assert obj.update_rate == 20  # Value from fixture
        assert obj.hal_comp is self.hal_comp  # Called init_hal_comp
        assert obj.count == 0  # Called setup_component in obj fixture
        obj.hal_comp.ready.assert_called_once()  # Called hal_comp.ready()

    def test_class_attrs(self):
        assert self.test_class.compname is not None

    def test_init_hal_comp(self, mock_hal_comp, mock_rclpy):
        # Test init_hal_comp() creates cached component object

        self.test_class._cached_objs.clear()  # Clean fixture
        assert "hal_comp" not in self.test_class._cached_objs  # Sanity

        obj = self.test_class(list())
        assert "hal_comp" in obj._cached_objs
        assert obj.hal_comp is mock_hal_comp

        with pytest.raises(AssertionError):
            obj.init_hal_comp()  # Already created

    def test_get_ros_param(self, obj):
        res = obj.get_ros_param("relative_tolerance", 42)
        assert res == 1e-9
        res = obj.get_ros_param("bogus_key", 88)
        assert res == 88
        res = obj.get_ros_param("bogus", 13)
        assert res == 13

    def test_run(self, obj):
        obj.run()
        self.node.create_timer.assert_called_with(
            1 / self.rosparams["update_rate"], obj.update
        )
        self.rclpy.spin.assert_called_with(self.node)
