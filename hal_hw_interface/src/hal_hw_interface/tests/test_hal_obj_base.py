# -*- coding: utf-8 -*-
import pytest

from hal_hw_interface.hal_obj_base import HalObjBase


class TestHalObjBase(object):
    compname = 'test_comp'
    test_class = HalObjBase

    @pytest.fixture
    def obj(self, mock_rospy):
        for key in list(self.test_class._cached_objs.keys()):
            self.test_class._cached_objs.pop(key)

        class BogusComp(HalObjBase):
            compname = self.compname

        BogusComp._cached_objs.pop('hal_comp', None)  # Clean fixture
        return BogusComp()

    def test_hal_obj_base_init_hal_comp(self, obj, mock_comp_obj):
        # Test init_hal_comp() creates cached component object
        obj.init_hal_comp()
        assert 'hal_comp' in obj._cached_objs
        assert obj._cached_objs['hal_comp'] is mock_comp_obj

    def test_hal_obj_base_init_hal_comp_no_compname(self):
        # Test init_hal_comp() on class with no 'compname' attribute
        class BogusNotComp(HalObjBase):
            pass

        with pytest.raises(RuntimeError):
            BogusNotComp().init_hal_comp()

    def test_hal_obj_base_init_hal_comp_twice(self, obj, mock_comp_obj):
        # Test init_hal_comp() multiple times
        obj.init_hal_comp()

        with pytest.raises(RuntimeError):
            obj.init_hal_comp()

    def test_hal_obj_base_hal_comp_property(self, obj, mock_comp_obj):
        # Test hal_comp property
        obj.init_hal_comp()
        assert obj.hal_comp is mock_comp_obj

    def test_hal_obj_base_hal_comp_not_initialized(self, obj, mock_comp_obj):
        # Test hal_comp property before init_hal_comp()

        with pytest.raises(RuntimeError):
            obj.hal_comp

    def test_hal_obj_base_get_ros_param(self, obj, mock_objs):
        test_params = dict(key1=42, key2='val2')
        gp = mock_objs['rospy_get_param']

        for key_short, set_val in test_params.items():
            # Mock return value
            key_long = '{}/{}'.format(self.compname, key_short)
            gp.set_key(key_long, set_val)

            # Call get_ros_param() and check
            get_val = obj.get_ros_param(key_short)
            gp.assert_called_with(key_long, None)
            assert get_val == set_val

        # Check default plumbing
        get_val = obj.get_ros_param('bogus_key', default='default_val')
        gp.assert_called_with(
            '{}/bogus_key'.format(self.compname), 'default_val'
        )
        assert get_val == 'default_val'
