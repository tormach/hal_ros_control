import pytest
from mock import MagicMock, PropertyMock, patch


@pytest.fixture()
def mock_hal_comp(request):
    # Mock hal.component() __getitem__() method (gets pin value)
    def get_pin_val(key):
        val = request.instance.pvals[key]
        print(f"Get HAL comp pin {key} = {val}")
        return val

    # Mock hal.component() __setitem__() method (sets pin value)
    def set_pin_val(key, val):
        print(f"Set HAL comp pin {key} = {val}")
        request.instance.pvals[key] = val

    # Mock hal.component().newpin() method & object
    def newpin(pname, ptype, pdir):
        request.instance.ptypes[pname] = ptype
        request.instance.pdirs[pname] = pdir
        request.instance.pvals[pname] = 0
        return MagicMock(f"mock HAL pin {pname}")

    # Mock hal.component() object
    hal_comp = MagicMock(name="mock_hal_comp_obj")
    hal_comp.getprefix.side_effect = lambda: request.instance.comp_name
    hal_comp.newpin.side_effect = newpin
    hal_comp.__getitem__.side_effect = get_pin_val
    hal_comp.__setitem__.side_effect = set_pin_val

    # Mock hal.component() method
    def component(name):
        request.instance.comp_name = name
        hal_comp.configure_mock(name=f"HAL comp {name}")
        return hal_comp

    # Mock hal import (for testing only)
    hal_component = MagicMock("hal.component", side_effect=component)
    hal = MagicMock("mock hal", component=hal_component)

    # Test instance storage attributes
    request.instance.ptypes = dict()  # HAL comp pin types
    request.instance.pdirs = dict()  # HAL comp pin directions
    request.instance.pvals = dict()  # HAL comp pin values
    request.instance.hal_comp = hal_comp
    request.instance.hal = hal

    patch("hal.component", side_effect=component).start()
    yield hal_comp
    patch.stopall()


@pytest.fixture()
def mock_rclpy(request):
    # rclpy.node.Node().declare_parameter():  looks up values in test
    # object `rosparams` attribute
    def declare_parameter_value_closure(name, default_value):
        def value():
            v = request.instance.rosparams.get(name, default_value)
            print(f"node.declare_parameter({name}).value = {v}")
            return v

        return value

    def declare_parameter(name, value=None):
        dp = MagicMock(name=f"mock_rclpy_declare_parameter({name})")
        pm = PropertyMock(
            side_effect=declare_parameter_value_closure(name, value)
        )
        type(dp).value = pm
        print(f"Created PropertyMock {pm}")
        return dp

    # rclpy.logging.get_logger():  info(), debug(), fatal() methods
    # print to stdout
    def get_logger_closure(name, level):
        def logger_method(msg_fmt, *args):
            print(f"{name}.{level}:  {msg_fmt % args}")

        return logger_method

    def get_logger(name):
        kwargs = dict()
        for level in ("info", "debug", "fatal"):
            kwargs[level] = MagicMock(
                side_effect=get_logger_closure(name, level)
            )
        logger = request.instance.loggers[name] = MagicMock(name, **kwargs)
        return logger

    # rclpy.node.Node().create_subscription():  create mock
    # rclpy.publisher.Publisher obj
    def create_publisher(msg_type, pub_topic):
        pub_pub = MagicMock(f"Publisher.publish {pub_topic} {str(msg_type)}")
        pub = request.instance.publishers[pub_topic] = MagicMock(
            f"Publisher {pub_topic} {str(msg_type)}",
            publish=pub_pub,
            msg_type=msg_type,
        )
        print(f"Created Publisher topic={pub_topic}")
        return pub

    # rclpy.node.Node().create_subscription():  create mock
    # rclpy.subscription.Subscription obj
    def create_subscription(msg_type, sub_topic, cb):
        sub = request.instance.subscriptions[sub_topic] = MagicMock(
            f"Subscription {sub_topic} {str(msg_type)}",
            msg_type=msg_type,
            cb=cb,
        )
        return sub

    # rclpy.node.Node().create_service():  create mock
    # rclpy.service.Service obj
    def create_service(srv_type, srv_name, cb):
        srv = request.instance.services[srv_name] = MagicMock(
            f"Service {srv_name} {str(srv_type)}", srv_type=srv_type, cb=cb
        )
        return srv

    # rclpy.create_node():  returns mock rclpy.node.Node() object; see
    # above declare_parameter() and get_logger() mock methods
    def create_node(node_name):
        node = request.instance.node = MagicMock(
            f"Node({node_name})",
            declare_parameter=MagicMock(side_effect=declare_parameter),
            get_logger=MagicMock(side_effect=lambda: get_logger(node_name)),
            create_publisher=MagicMock(side_effect=create_publisher),
            create_subscription=MagicMock(side_effect=create_subscription),
            create_service=MagicMock(side_effect=create_service),
        )
        return node

    # rclpy.utilities.get_default_context().ok():  returns True if
    # test object `context_ok_cycles--` attr (default 3) > 0
    def context_ok():
        if request.instance.context_ok_cycles > 0:
            request.instance.context_ok_cycles -= 1
            return True
        else:
            return False

    # Mock context
    ctx = MagicMock(name="rclpy.context.Context")
    ctx.ok.side_effect = context_ok

    # Mock rclpy
    rclpy = MagicMock(name="rclpy")
    rclpy.create_node.side_effect = create_node
    rclpy.utilities.get_default_context.return_value = ctx
    rclpy.logging.get_logger.side_effect = get_logger
    # These don't really need to do anything:  spin_once, init

    # Test instance storage attributes
    request.instance.rosparams = dict()  # Rosparam values by key
    request.instance.loggers = dict()  # Loggers by name
    request.instance.publishers = dict()  # Publishers by name
    request.instance.subscriptions = dict()  # Subscriptions by name
    request.instance.services = dict()  # Services by name
    request.instance.node = None  # rclpy.node.Node object
    if not hasattr(request.instance, "context_ok_cycles"):
        request.instance.context_ok_cycles = 3  # Num. cycles ok() is True
    request.instance.context = ctx  # rclpy.context.Context
    request.instance.rclpy = rclpy  # rclpy import

    patch("hal_hw_interface.hal_mgr.rclpy", rclpy).start()
    patch("hal_hw_interface.hal_obj_base.rclpy", rclpy).start()
    patch("hal_hw_interface.loadrt_local.rclpy", rclpy).start()
    patch("hal_hw_interface.ros_hal_component.rclpy", rclpy).start()

    yield rclpy

    patch.stopall()


@pytest.fixture()
def sentinel_obj_cb(request):
    so = request.instance.sentinel_obj = MagicMock("Sentinel object")
    socb = request.instance.sentinel_obj_cb = MagicMock(
        "Sentinel object callable", side_effect=lambda: so
    )
    yield socb
