# -*- coding: utf-8 -*-
from hal_hw_interface.redis_store_hal_pin import RedisStoreHalPin

# Borrow tests from ros_hal_pin
from test_ros_hal_pin import TestRosHalPin, HalPinDir


class TestRedisStoreHalPin(TestRosHalPin):
    default_hal_dir = HalPinDir('OUT')
    test_class = RedisStoreHalPin

    def redis_param_key(self, obj):
        return "{}/{}".format(obj.compname, obj.pin_name)

    def set_mock_redis_param(self, obj, param):
        if isinstance(param, dict):
            value = self.other_value(obj, param)
        else:
            value = param
        obj._redis_config.set_key(self.redis_param_key(obj), value)
        return value

    def test_redis_store_pin_param_key(self, obj):
        assert obj._redis_param_key == self.redis_param_key(obj)

    def test_redis_store_pin_client(self, mock_redis_client_obj, mock_comp_obj):
        # Check that redis client is created and cached
        self.setup_hal_obj_base(mock_comp_obj)
        c1 = self.test_class('test_redis_pin', 'FLOAT')._redis_config
        assert c1 is mock_redis_client_obj
        c2 = self.test_class('test_redis_pin2', 'FLOAT')._redis_config
        assert c1 is c2

    def test_redis_store_pin_set_pin_from_redis(
        self, obj, data, mock_comp_obj, mock_redis_client_obj
    ):
        # Fake a redis param value and set the pin from redis
        test_val = self.set_mock_redis_param(obj, data)
        obj_val = obj.set_pin_from_redis()
        # Check that redis was read
        mock_redis_client_obj.get_param.assert_called_once_with(
            '{}/{}'.format(self.compname, self.obj_test_name(obj))
        )
        # Check the returned value
        assert obj_val == test_val
        # Check the HAL pin value
        mock_comp_obj.__setitem__.assert_called_with(
            self.obj_test_name(obj), test_val
        )

    def test_redis_store_pin_set_pin_no_defaults(self, all_patches):
        (
            mock_comp_obj,
            mock_rospy,
            mock_objs,
            mock_redis_client_obj,
        ) = all_patches
        # Fake a redis param value and set the pin from redis
        obj = self.test_class('unconfigured_redis_pin', 'FLOAT')
        self.setup_hal_obj_base(mock_comp_obj)
        self.set_mock_redis_param(obj, None)
        mock_redis_client_obj.reset_mock()
        # Call set_pin_from_redis() and check that the pin was NOT set
        print("--------- Calling method")
        obj.set_pin_from_redis()
        print(mock_comp_obj.__setitem__.mock_calls)
        mock_comp_obj.__setitem__.assert_not_called()

    def test_redis_store_pin_set_redis_from_pin(
        self, obj, data, mock_redis_client_obj
    ):
        # Fake a HAL pin value, set redis key, clean up
        test_val = self.set_pin(obj, data)
        self.set_mock_redis_param(obj, data)
        mock_redis_client_obj.reset_mock()
        # Call the method
        print("--------- Calling method")
        obj_val = obj.set_redis_from_pin()
        # Check the returned value
        assert obj_val == test_val
        # Check redis was written to
        mock_redis_client_obj.set_param.assert_called_once_with(
            '{}/{}'.format(self.compname, self.obj_test_name(obj)), test_val
        )
        # Check if log message was printed
        print(mock_redis_client_obj.get_param.mock_calls)
        if data['changed']:
            # Call get_param to check value and again to print log
            assert mock_redis_client_obj.get_param.call_count == 2
        else:
            # Only call get_param once to check value, not again to print log
            assert mock_redis_client_obj.get_param.call_count == 1

    def test_redis_store_pin_set_redis_from_pin_first_time(
        self, obj, data, mock_redis_client_obj
    ):
        # Fake a HAL pin value, set redis key to None, clean up
        test_val = self.set_pin(obj, data)
        self.set_mock_redis_param(obj, None)
        mock_redis_client_obj.reset_mock()
        # Call the method
        obj_val = obj.set_redis_from_pin()
        # Check the returned value
        assert obj_val == test_val
        # Check redis was written to
        mock_redis_client_obj.set_param.assert_called_once_with(
            '{}/{}'.format(self.compname, self.obj_test_name(obj)), test_val
        )
        # Check the log message was not printed
        print(mock_redis_client_obj.get_param.mock_calls)
        assert mock_redis_client_obj.get_param.call_count == 1
