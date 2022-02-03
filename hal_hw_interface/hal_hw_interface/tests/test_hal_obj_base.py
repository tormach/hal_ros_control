import pytest

from hal_hw_interface.hal_obj_base import HalObjBase


class TestHalObjBase:
    comp_name = "test_comp"
    test_class = HalObjBase

    @pytest.fixture
    def obj(self, mock_rclpy, mock_hal_comp):
        class BogusComp(HalObjBase):
            compname = self.comp_name

        BogusComp._cached_objs.clear()  # Clean fixture
        obj = BogusComp()
        obj.init_ros_node(list())
        yield obj

    def test_logger(self, obj):
        obj.logger.reset_mock()
        for i in ("info", "debug", "fatal"):
            method = getattr(obj.logger, i)
            msg = f"{i} foo"
            method(msg)
            method.assert_called_once()
            method.assert_called_with(msg)

    def test_hal_comp(self, obj, sentinel_obj_cb):
        with pytest.raises(AssertionError):
            obj.hal_comp

        obj._cached_objs["hal_comp"] = self.sentinel_obj
        assert obj.hal_comp is self.sentinel_obj

    def test_init_ros_node(self, obj):
        # Check ROS communications initialized
        self.rclpy.init.assert_called_once()

        # Check node
        self.rclpy.create_node.assert_called_once()
        self.rclpy.create_node.assert_called_with(self.comp_name)
        assert hasattr(obj, "node")
        assert obj.node is self.node

        # Check context
        self.rclpy.utilities.get_default_context.assert_called_once()
        assert hasattr(obj, "node_context")
        assert obj.node_context is self.context

    def test_get_ros_param(self, obj):
        rosparams = dict(key1=42, key2="val2")

        for key, val in rosparams.items():
            # Mock return value
            self.rosparams[key] = val

            # Call get_ros_param() and check
            assert obj.get_ros_param(key) == val

            # Call get_ros_param() with default and check
            self.node.declare_parameter.reset_mock()
            assert obj.get_ros_param(key, default=17) == val
            self.node.declare_parameter.assert_not_called()

        # Test default
        assert obj.get_ros_param("bogus", 13) == 13
        self.node.declare_parameter.assert_called_with("bogus", 13)
        self.node.declare_parameter.reset_mock()
        assert obj.get_ros_param("bogus") == 13
        self.node.declare_parameter.assert_not_called()

    def test_shutdown_callbacks(self, obj, sentinel_obj_cb):
        assert "shutdown_cbs" not in obj._cached_objs  # Sanity

        # Add a shutdown callback
        obj.add_shutdown_callback(sentinel_obj_cb, 42)
        assert "shutdown_cbs" in obj._cached_objs
        shutdown_cbs = obj._cached_objs["shutdown_cbs"]
        assert len(shutdown_cbs) == 1
        assert 42 in shutdown_cbs
        assert shutdown_cbs[42] == sentinel_obj_cb

        # Add a second shutdown callback with same prio
        obj.add_shutdown_callback(sentinel_obj_cb, 42)
        assert len(shutdown_cbs) == 2
        assert 42.1 in shutdown_cbs
        assert shutdown_cbs[42.1] == sentinel_obj_cb

        # Run shutdown callbacks
        obj._run_shutdown_cbs()
        assert sentinel_obj_cb.call_count == 2
        # FIXME test callback ordering
