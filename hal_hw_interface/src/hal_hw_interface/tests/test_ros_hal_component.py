# -*- coding: utf-8 -*-
import pytest
from hal_hw_interface.ros_hal_component import RosHalComponent


class TestRosHalComponent(object):
    test_class = RosHalComponent

    @pytest.fixture
    def obj(self, mock_comp_obj, mock_rospy, mock_objs):
        for key in list(self.test_class._cached_objs.keys()):
            self.test_class._cached_objs.pop(key)

        # Simplest test case
        class StubComp(self.test_class):
            compname = 'stub'

            def setup_component(self):
                self.initialized = True
                self.count = 0

            def update(self):
                self.count += 1

        gp = mock_objs['rospy_get_param']
        gp.set_key('stub/update_rate', 20)
        gp.set_key('stub/relative_tolerance', 1e-9)
        gp.set_key('stub/absolute_tolerance', 1e-9)

        return StubComp()

    def test_ros_hal_component_attrs(self, obj):
        assert obj.compname == 'stub'

    def test_ros_hal_component_init(
        self, obj, mock_rospy, mock_comp_obj, mock_objs
    ):
        '''Test RosHalComponent.__init__()
        '''
        # ROS node initialized
        mock_rospy['init_node'].assert_called_with(obj.compname)
        # obj.rate was created from rospy.Rate with param value
        mock_objs['rospy_get_param'].assert_called_once_with(
            'stub/update_rate', 10
        )
        assert obj.update_rate == 20
        mock_objs['rospy_Rate'].assert_called_once_with(20)
        assert obj.rate is mock_objs['rospy_Rate_obj']
        # Assert obj.hal_comp is a HAL component
        mock_objs['hal_comp'].assert_called_once_with('stub')
        assert obj.hal_comp == mock_comp_obj
        # Assert StubComp.setup_component() called
        assert getattr(obj, 'initialized', False) is True
        # Assert HAL component initialized
        obj.hal_comp.ready.assert_called_once_with()

    def test_ros_hal_component_get_param(self, obj, mock_objs):
        '''Test get_param()
        '''
        res = obj.get_param('relative_tolerance', 42)
        assert res == 1e-9
        res = obj.get_param('bogus_key', 88)
        assert res == 88

    def test_ros_hal_component_run(self, obj, mock_objs):
        '''Test run() (fixture loops three times); should call update() and
        rate.sleep()
        '''
        obj.run()
        assert obj.count == 3
        assert mock_objs['rospy_Rate_obj'].sleep.call_count == 3
