import pytest
from mock import MagicMock, PropertyMock, patch


@pytest.fixture()
def mock_hal_comp(request):
    # Mock hal.component() __getitem__() method (gets pin value)
    def get_pin_val(key):
        val = request.instance.pvals[key]
        print(
            f"  mock_hal_comp:  {request.instance.comp_name}"
            f".__getitem__('{key}') => {val}"
        )
        return val

    # Mock hal.component() __setitem__() method (sets pin value)
    def set_pin_val(key, val):
        print(
            f"  mock_hal_comp:  {request.instance.comp_name}"
            f".__getitem__('{key}') => {val}"
        )
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
    # Instance must define `rclpy_patches` list of classes to patch
    assert hasattr(request.instance, "rclpy_patches")

    # rclpy.node.Node().declare_parameter():  looks up values in test
    # object `rosparams` attribute
    def parameter_value_property(name, default_value):
        default_sentinel = MagicMock(name="default")

        def value(v=default_sentinel):
            if v is not default_sentinel:
                request.instance.rosparams[name] = v
                print(f"  mock_rclpy:  set parameter({name}).value = {v}")
            else:
                v = request.instance.rosparams.get(name, default_value)
                print(f"  mock_rclpy:  get parameter({name}).value => {v}")
                return v

        value_property = PropertyMock(f"param {name}.value", side_effect=value)
        return value_property

    def declare_parameter(name, value=None):
        if name in request.instance.rosparam_decls:
            raise RuntimeError(
                f"Node.declare_parameter({name}):  already declared"
            )
        dp = MagicMock(name=f"mock_rclpy_declare_parameter({name})")
        type(dp).value = parameter_value_property(name, value)
        request.instance.rosparam_decls[name] = dp
        print(f"  mock_rclpy:  node.declare_parameter(name, value={value})")
        return dp

    def has_parameter(name):
        res = name in request.instance.rosparam_decls
        print(f"  mock_rclpy:  node.has_parameter({name}) = {res}")
        return res

    # rclpy.logging.get_logger():  info(), debug(), fatal() methods
    # print to stdout
    def get_logger_closure(name, level):
        def logger_method(msg_fmt, *args):
            print(f"logger:  {name}.{level}:  {msg_fmt % args}")

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
    def create_publisher(msg_type, pub_topic, qos_profile):
        desc = f"'{pub_topic}' msg_type={msg_type.__name__}"
        pub_pub = MagicMock(f"Publisher.publish {desc}")
        pub = request.instance.publishers[pub_topic] = MagicMock(
            name=f"Publisher {desc}",
            publish=pub_pub,
            msg_type=msg_type,
        )
        print(
            f"  mock_rclpy:  node.create_publisher("
            f"{msg_type}, {pub_topic}, {qos_profile}"
        )
        return pub

    # rclpy.node.Node().create_subscription():  create mock
    # rclpy.subscription.Subscription obj
    def create_subscription(msg_type, sub_topic, cb):
        desc = f"'{sub_topic}' msg_type={msg_type.__name__}"
        sub = request.instance.subscriptions[sub_topic] = MagicMock(
            name=f"Subscription {desc}",
            msg_type=msg_type,
            cb=cb,
        )
        return sub

    # rclpy.node.Node().create_service():  create mock
    # rclpy.service.Service obj
    def create_service(srv_type, srv_name, cb):
        desc = f"'{srv_name}' srv_type={srv_type.__name__}"
        srv = request.instance.services[srv_name] = MagicMock(
            name=f"Service {desc}", srv_type=srv_type, cb=cb
        )
        return srv

    # rclpy.create_node():  returns mock rclpy.node.Node() object; see
    # above declare_parameter() and get_logger() mock methods
    def create_node(node_name, **kwargs):
        kwargs_str = ", ".join([f"{k}={v}" for k, v in kwargs.items()])
        print(f"  mock_rclpy:  create_node({node_name}, {kwargs_str})")
        node = request.instance.node = MagicMock(
            f"Node({node_name}, {kwargs_str})",
            declare_parameter=MagicMock(side_effect=declare_parameter),
            has_parameter=MagicMock(side_effect=has_parameter),
            get_logger=MagicMock(side_effect=lambda: get_logger(node_name)),
            create_publisher=MagicMock(side_effect=create_publisher),
            create_subscription=MagicMock(side_effect=create_subscription),
            create_service=MagicMock(side_effect=create_service),
            create_timer=MagicMock(name="create_timer"),
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
    # These don't really need to do anything:  spin, init

    # Test instance storage attributes
    request.instance.rosparams = dict()  # Rosparam values by key
    request.instance.rosparam_decls = dict()  # Declared rosparams
    request.instance.loggers = dict()  # Loggers by name
    request.instance.publishers = dict()  # Publishers by name
    request.instance.subscriptions = dict()  # Subscriptions by name
    request.instance.services = dict()  # Services by name
    request.instance.node = None  # rclpy.node.Node object
    if not hasattr(request.instance, "context_ok_cycles"):
        request.instance.context_ok_cycles = 3  # Num. cycles ok() is True
    request.instance.context = ctx  # rclpy.context.Context
    request.instance.rclpy = rclpy  # rclpy import

    for p in request.instance.rclpy_patches:
        patch(p, rclpy).start()

    yield rclpy

    patch.stopall()


@pytest.fixture()
def sentinel_obj_cb(request):
    so = request.instance.sentinel_obj = MagicMock("Sentinel object")
    socb = request.instance.sentinel_obj_cb = MagicMock(
        "Sentinel object callable", side_effect=lambda: so
    )
    yield socb
