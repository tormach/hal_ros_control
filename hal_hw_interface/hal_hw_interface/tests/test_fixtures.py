from pprint import pformat

# Test keys and values
keys1 = dict(pin1=True, pin2=0.009, pin3=-9)

# More test keys and values, an overlapping set
keys2 = dict(pin1=False, pin2=1.88e42, pin4=0)


class TestFixtures:
    comp_name = "test_fixtures_comp"

    def test_mock_hal_comp_fixture(self, mock_hal_comp):
        # Test hal.component returns mock_hal_comp
        assert self.hal.component("foo_comp") is mock_hal_comp
        assert self.comp_name == "foo_comp"

        # Set each pin (with out-of-band method) and check
        self.pvals = keys1.copy()
        for name, value in keys1.items():
            assert mock_hal_comp[name] == value

        # Recheck
        for name, value in keys1.items():
            assert mock_hal_comp[name] == value

        # Set each pin and check
        for name, value in keys2.items():
            mock_hal_comp[name] = value
            assert mock_hal_comp[name] == value

        # Recheck everything
        pins = keys1.copy()
        pins.update(keys2)
        for name, value in pins.items():
            assert mock_hal_comp[name] == value

    def test_mock_rospy_fixture(self, mock_rclpy, sentinel_obj_cb):
        rclpy = mock_rclpy
        assert rclpy is self.rclpy

        # Test rclpy.create_node()
        node = rclpy.create_node("foo_node")
        assert node is self.node

        # Test mock node.declare_parameter()
        self.rosparams["foo"] = 1
        self.rosparams["bar"] = 2
        print("self.rosparams", pformat(self.rosparams))
        assert self.node.declare_parameter("foo").value == 1
        assert self.node.declare_parameter("bar").value == 2
        self.rosparams["baz"] = 3
        assert self.node.declare_parameter("foo").value == 1
        assert self.node.declare_parameter("bar").value == 2
        assert self.node.declare_parameter("baz").value == 3

        # Test mock node.get_logger()
        logger = node.get_logger()
        assert "foo_node" in self.loggers
        assert logger is self.loggers["foo_node"]
        # - These will print to stdout
        logger.info("info foo")
        logger.debug("debug foo")
        logger.fatal("fatal foo")

        # Test node.create_publisher()
        pub = node.create_publisher(int, "foo_publisher")
        assert "foo_publisher" in self.publishers
        assert pub is self.publishers["foo_publisher"]
        assert pub.msg_type is int

        # Test node.create_subscription()
        sub = node.create_subscription(int, "foo_subscription", sentinel_obj_cb)
        assert "foo_subscription" in self.subscriptions
        assert sub is self.subscriptions["foo_subscription"]
        assert sub.msg_type is int
        assert sub.cb() is self.sentinel_obj

        # Test node.create_service()
        svc = node.create_service(float, "foo_service", sentinel_obj_cb)
        assert "foo_service" in self.services
        assert svc is self.services["foo_service"]
        assert svc.srv_type is float
        assert svc.cb() is self.sentinel_obj

        # Test context ok()
        assert self.context_ok_cycles == 3  # Default
        self.context_ok_cycles = 2
        ctx = rclpy.utilities.get_default_context()
        assert ctx is self.context
        assert ctx.ok()
        assert self.context_ok_cycles == 1
        assert ctx.ok()
        assert self.context_ok_cycles == 0
        assert not ctx.ok()
        assert self.context_ok_cycles == 0
        assert not ctx.ok()
