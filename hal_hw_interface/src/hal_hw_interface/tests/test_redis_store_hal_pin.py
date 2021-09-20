# -*- coding: utf-8 -*-
import pytest
from hal_hw_interface.redis_store_hal_pin import RedisStoreHalPin

# Borrow tests from ros_hal_pin
from test_ros_hal_pin import TestRosHalPin, HalPinDir


class TestRedisStoreHalPin(TestRosHalPin):
    default_hal_dir = HalPinDir('OUT')
    test_class = RedisStoreHalPin

    @pytest.fixture(params=TestRosHalPin.obj_cases)
    def obj(self, request, mock_comp_obj, mock_rospy, mock_redis_client_obj):
        self.setup_hal_obj_base(mock_comp_obj)
        mock_comp_obj.setprefix(self.compname)
        attrs = dict()
        params = request.param.copy()
        params.setdefault('hal_dir', self.default_hal_dir)
        name = params.pop('name')
        attr_names = ['hal_comp', 'hal_type', 'hal_dir', 'msg_type']
        attr_names += self.extra_attrs
        for attr_name in attr_names:
            if attr_name in params:
                attrs[attr_name] = params.pop(attr_name)
        obj = self.test_class(name, **attrs)
        obj._p = request.param  # Send test params in
        mock_redis_client_obj.get_param.reset_mock()  # Called during setup
        return obj

    def redis_param_key(self, obj):
        return "{}/{}".format(obj.compname, obj.pin_name)

    def set_mock_redis_param(self, obj, param):
        if isinstance(param, dict):
            value = self.other_value(obj, param)
        else:
            value = param
        obj._redis_config.set_key(self.redis_param_key(obj), value)
        return value

    def test_redis_store_pin_key(self, obj):
        assert obj.key == self.redis_param_key(obj)

    def test_redis_store_pin_redis_config(
        self, mock_redis_client_obj, mock_comp_obj, mock_rospy
    ):
        # Check that redis client is created and cached
        self.setup_hal_obj_base(mock_comp_obj)
        c1 = self.test_class('test_redis_pin', 'FLOAT')._redis_config
        assert c1 is mock_redis_client_obj
        c2 = self.test_class('test_redis_pin2', 'FLOAT')._redis_config
        assert c1 is c2

    def test_redis_store_pin_update_fm_redis(self, obj):
        obj._update_fm_redis(obj.key, 42)
        print(self.pin_values)
        assert self.pin_values[obj.pin_name] == 42
        assert obj._prev_pin_val == 42
        assert obj._prev_redis_val == 42
        # When key doesn't match, no change
        obj._update_fm_redis('foo', 13)
        assert self.pin_values[obj.pin_name] == 42

    def test_redis_store_pin_ros_init(
        self, obj, data, mock_comp_obj, mock_redis_client_obj
    ):
        # obj._redis_config.on_update_received = list()  # Clear out setup entry
        # Fake a redis param value & run function under test
        test_val = self.set_mock_redis_param(obj, data)
        self.set_pin(obj, data['pin_value'])
        obj._ros_init()

        if str(obj.hal_dir) != 'HAL_IN':
            # Check that redis was read
            print(
                'get_param calls:', mock_redis_client_obj.get_param.mock_calls
            )
            mock_redis_client_obj.get_param.assert_called_once_with(obj.key)
            # Check the returned value
            assert obj._prev_pin_val == test_val
            assert obj._prev_redis_val == test_val
            assert self.pin_values[obj.pin_name] == test_val
            # Check the callback list
            assert (
                obj._redis_config.on_update_received[-1] == obj._update_fm_redis
            )

        else:  # HAL_IN pins
            assert obj._prev_pin_val == self.pin_values[obj.pin_name]
            assert obj._prev_redis_val is None

    def test_redis_store_pin_set_redis_from_pin(
        self, obj, data, mock_redis_client_obj
    ):
        # Fake values
        test_val = self.set_pin(obj, data)
        last_val = self.set_last_value(obj, data)
        obj._prev_pin_val = obj._prev_redis_val = last_val

        # Call the method
        print("--------- Calling method")
        obj.update()

        if str(obj.hal_dir) == 'HAL_OUT':
            obj._redis_config.set_param.assert_not_called()
            return

        if test_val == last_val:
            return

        # Check the new value
        assert obj._prev_pin_val == test_val
        assert obj._prev_redis_val == test_val
