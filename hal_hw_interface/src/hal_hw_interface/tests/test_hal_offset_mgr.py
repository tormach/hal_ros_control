# -*- coding: utf-8 -*-
import pytest

from hal_hw_interface.ros_hal_pin import RosHalPin
from hal_hw_interface.hal_offset_mgr import HalOffsetMgrPin, HalOffsetMgr

# Borrow tests from redis_store_hal_pin
from test_ros_hal_pin import HalPinDir, HalPinType
from test_redis_store_hal_pin import TestRedisStoreHalPin


class TestHalOffsetMgrPin(TestRedisStoreHalPin):
    default_hal_type = HalPinType('FLOAT')
    default_hal_dir = HalPinDir('OUT')
    test_class = HalOffsetMgrPin

    newpin_calls = 2

    def obj_test_name(self, obj):  # Override base test
        if isinstance(obj, self.test_class):
            return self.get_obj_test_param(obj, 'name') + '_offset'
        else:
            return super(TestHalOffsetMgrPin, self).obj_test_name(obj)

    def test_ros_hal_pin_attrs(self, obj, mock_comp_obj):  # Override base test
        '''Test that attributes are set as expected, including defaults
        '''
        # Call original test against offset pin
        super(TestHalOffsetMgrPin, self).test_ros_hal_pin_attrs(
            obj, mock_comp_obj
        )

        # Test complementary '_fb_in' newpin() call
        pos_pin = obj.hal_fb_in_pin
        assert pos_pin.name == obj.name + '_fb-in'
        assert pos_pin.hal_comp is mock_comp_obj
        assert type(pos_pin.hal_type) is HalPinType
        assert pos_pin.hal_type == HalPinType(self.hal_type(obj))
        assert type(pos_pin.hal_dir) is HalPinDir
        assert pos_pin.hal_dir == HalPinDir('IN')

    def test_ros_hal_pin_newpin(self, obj, mock_comp_obj):  # Override base test
        '''Test that hal.component.newpin() is called
        '''
        # Call original test against offset pin
        super(TestHalOffsetMgrPin, self).test_ros_hal_pin_newpin(
            obj, mock_comp_obj
        )

        # Test complementary '_fb_in' newpin() call
        mock_comp_obj.newpin.assert_any_call(
            obj.name + '_fb-in', HalPinType(self.hal_type(obj)), HalPinDir('IN')
        )

    def test_hal_offset_mgr_pin_zero_joint(
        self, obj, data, mock_comp_obj, mock_redis_client_obj
    ):
        '''Test zero_joint() sets offset pin and redis from position input pin
        '''
        # Add obj test case data to position input pin object
        pos_pin_case = obj._p.copy()
        pos_pin_case['name'] = obj.name + '_fb-in'
        obj.hal_fb_in_pin._p = pos_pin_case

        # Fake offset and position input values and run zero_joint()
        new_val = self.set_pin(obj.hal_fb_in_pin, self.other_value(obj, data))
        old_val = self.set_pin(obj, data)
        assert mock_comp_obj[obj.hal_fb_in_pin.name] == new_val  # Sanity check
        assert mock_comp_obj[obj.pin_name] == old_val  # Sanity check
        print("Calling zero_joint()")
        updated_val = obj.zero_joint()

        # Check the pos pin was read correctly
        assert updated_val == new_val
        # Check the HAL pin value was written correctly
        assert mock_comp_obj[self.obj_test_name(obj)] == new_val
        # Check redis was written to
        mock_redis_client_obj.set_param.assert_called_once_with(
            '{}/{}'.format(self.compname, self.obj_test_name(obj)), new_val
        )

    def test_hal_offset_mgr_load_offset(self, obj, data):
        # Add obj test case data to position input pin object
        pos_pin_case = obj._p.copy()
        pos_pin_case['name'] = obj.name + '_fb-in'
        obj.hal_fb_in_pin._p = pos_pin_case

        # Fake redis and pin values and run load_offset()
        redis_val = self.set_mock_redis_param(obj, data)
        pin_val = self.set_pin(obj, data)
        if data['changed']:  # Sanity check
            assert pin_val != redis_val
        ret_val = obj.load_offset()

        # Check the HAL pin and return values are correct
        assert obj.get_pin() == redis_val
        assert ret_val == redis_val


class TestHalOffsetMgr(object):
    test_class = HalOffsetMgr
    test_pin_class = HalOffsetMgrPin
    compname = 'hal_offset_mgr'

    joint_names = ['joint_1', 'joint_2']
    num_pins = len(joint_names)
    offset_pins = ['{}_offset'.format(j) for j in joint_names]
    fb_pins = ['{}_fb-in'.format(j) for j in joint_names]
    redis_keys = ['%s/%s' % (compname, p) for p in offset_pins]
    test_data = zip(offset_pins, fb_pins, redis_keys, (42, 13))

    @pytest.fixture
    def obj(self, mock_comp_obj, mock_rospy, mock_redis_client_obj, mock_objs):
        for key in list(self.test_class._cached_objs.keys()):
            self.test_class._cached_objs.pop(key)

        gp = mock_objs['rospy_get_param']
        gp.set_key('hardware_interface/joints', self.joint_names)

        mock_comp_obj.set_prefix(self.compname)
        return self.test_class()

    def test_hal_offset_mgr_setup_component(self, obj):
        # Check offset_pins list
        assert len(obj.offset_pins) == len(self.joint_names)
        for op in obj.offset_pins:
            assert isinstance(op, self.test_pin_class)
            assert op.hal_type == HalPinType('FLOAT')
            assert op.hal_dir == HalPinDir('OUT')

        # Check other pins
        for data in (
            ('zero_all_joints', 'IO'),
            ('load_params', 'IO'),
            ('enable', 'IN'),
        ):
            name, hal_dir = data
            assert hasattr(obj, name)
            pin = getattr(obj, name)
            assert isinstance(pin, RosHalPin)
            assert pin.hal_type == HalPinType('BIT')
            assert pin.hal_dir == HalPinDir(hal_dir)

    def test_hal_offset_mgr_update_no_cmd(self, obj):
        # Test no command

        obj.hal_comp.set_pin('load_params', 0)
        obj.hal_comp.set_pin('zero_all_joints', 0)
        obj.hal_comp.set_pin('enable', 0)
        obj.hal_comp.reset_mock()
        obj.update()
        print(obj.hal_comp.mock_calls)

        # Cmd pins read, but nothing set
        assert obj.hal_comp.__getitem__.call_count == 2
        obj.hal_comp.__getitem__.assert_any_call('load_params')
        obj.hal_comp.__getitem__.assert_any_call('zero_all_joints')
        obj.hal_comp.__setitem__.assert_not_called()

    def test_hal_offset_mgr_update_load_params(
        self, obj, mock_redis_client_obj, mock_comp_obj
    ):
        # Test load_params command

        mock_comp_obj.setprefix(self.compname)
        obj.hal_comp.set_pin('load_params', 1)
        obj.hal_comp.set_pin('zero_all_joints', 0)
        obj.hal_comp.set_pin('enable', 0)
        for pin_o, pin_fb, redis_key, val in self.test_data:
            mock_redis_client_obj.set_key(redis_key, val)
        obj.hal_comp.reset_mock()
        obj.update()
        print("obj.hal_comp calls:")
        print(obj.hal_comp.mock_calls)
        print("mock_redis_client calls:")
        print(mock_redis_client_obj.mock_calls)

        # total n+1 pin writes, n param reads, no param writes
        assert obj.hal_comp.__setitem__.call_count == self.num_pins + 1
        assert mock_redis_client_obj.get_param.call_count == self.num_pins
        mock_redis_client_obj.set_param.assert_not_called
        # load_params read & written
        obj.hal_comp.__getitem__.assert_any_call('load_params')
        obj.hal_comp.__setitem__.assert_any_call('load_params', False)
        # offsets written
        for pin_o, pin_fb, redis_key, val in self.test_data:
            mock_redis_client_obj.get_param.assert_any_call(redis_key)
            assert mock_redis_client_obj.get_param(redis_key) == val
            obj.hal_comp.__setitem__.assert_any_call(pin_o, val)

    def test_hal_offset_mgr_update_zero_all_joints_disabled(
        self, obj, mock_redis_client_obj, mock_comp_obj
    ):
        # Test zero_all_joints command when disabled

        mock_comp_obj.setprefix(self.compname)
        obj.hal_comp.set_pin('load_params', 0)
        obj.hal_comp.set_pin('zero_all_joints', 1)
        obj.hal_comp.set_pin('enable', 0)
        for pin_o, pin_fb, redis_key, val in self.test_data:
            obj.hal_comp.set_pin(pin_fb, val)
        obj.hal_comp.reset_mock()
        obj.update()
        print("obj.hal_comp calls:")
        print(obj.hal_comp.mock_calls)
        print("mock_redis_client calls:")
        print(mock_redis_client_obj.mock_calls)

        # total n+1 pin writes, n params read, 0 params written
        assert obj.hal_comp.__setitem__.call_count == self.num_pins + 1
        assert mock_redis_client_obj.set_param.call_count == 2
        mock_redis_client_obj.set_param.assert_not_called
        # zero_all_joints pin read & written
        obj.hal_comp.__getitem__.assert_any_call('zero_all_joints')
        obj.hal_comp.__setitem__.assert_any_call('zero_all_joints', False)
        # enable read
        obj.hal_comp.__getitem__.assert_any_call('enable')
        # offsets read from fb pins, written to offset pins and redis
        for pin_o, pin_fb, redis_key, val in self.test_data:
            obj.hal_comp.__getitem__.assert_any_call(pin_fb)
            obj.hal_comp.__setitem__.assert_any_call(pin_o, val)
            assert obj.hal_comp[pin_o] == val
            mock_redis_client_obj.set_param.assert_any_call(redis_key, val)
            assert mock_redis_client_obj.get_param(redis_key) == val

    def test_hal_offset_mgr_update_zero_all_joints_enabled(
        self, obj, mock_redis_client_obj
    ):
        # Test zero_all_joints command when enabled

        obj.hal_comp.set_pin('load_params', 0)
        obj.hal_comp.set_pin('zero_all_joints', 1)
        obj.hal_comp.set_pin('enable', 1)
        obj.hal_comp.reset_mock()
        obj.update()
        print("obj.hal_comp calls:")
        print(obj.hal_comp.mock_calls)
        print("mock_redis_client calls:")
        print(mock_redis_client_obj.mock_calls)

        # total 1 pin write
        assert obj.hal_comp.__setitem__.call_count == 1
        mock_redis_client_obj.set_param.assert_not_called
        mock_redis_client_obj.set_param.assert_not_called
        # zero_all_joints pin read & written
        obj.hal_comp.__getitem__.assert_any_call('zero_all_joints')
        obj.hal_comp.__setitem__.assert_any_call('zero_all_joints', False)
        # enable read
        obj.hal_comp.__getitem__.assert_any_call('enable')

    def test_hal_offset_mgr_shutdown_component(
        self, obj, mock_redis_client_obj
    ):
        # Test that redis client disconnects
        obj.offset_pins[0]._redis_config  # Autovivify client
        obj.shutdown_component()
        print("mock_redis_client calls:")
        print(mock_redis_client_obj.mock_calls)
        assert mock_redis_client_obj.stop.call_count == 1
