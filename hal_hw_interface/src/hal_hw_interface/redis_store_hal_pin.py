# -*- coding: utf-8 -*-

"""

   :synopsis: HAL pin object connected to `redis_store` parameter
     server for use in :py:mod:`hal_hw_interface.ros_hal_component`

   .. moduleauthor:: John Morris <john@dovetail-automata.com>

   .. inheritance-diagram::
        hal_hw_interface.redis_store_hal_pin.RedisStoreHalPin
"""

import rospy

# Mock patching breaks with `from redis_store import ConfigClient`; why?
import redis_store.config as redis_config

from hal_hw_interface.ros_hal_pin import RosHalPin


class RedisStoreHalPin(RosHalPin):
    '''HAL pin attached to :code:`redis_store` ROS package's parameter
    server

    This HAL pin's value may be read from and written to a
    :code:`redis_store` parameter server.  The default key name is
    :code:`<compname>/<name>`.

    :param name: The HAL pin name
    :type name: str
    :param hal_type: HAL pin data type, one of :code:`['BIT', 'U32',
      'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of :code:`['IN', 'OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    '''

    # Usually (?) read output pin default value from redis
    _default_hal_dir = 'OUT'

    @property
    def _redis_config(self):
        if 'redis_config_client' not in self._cached_objs:
            # Autovivify
            c = redis_config.ConfigClient()
            self._cached_objs['redis_config_client'] = c
        return self._cached_objs['redis_config_client']

    @property
    def _redis_param_key(self):
        return "{}/{}".format(self.compname, self.pin_name)

    def set_pin_from_redis(self, default=None):
        '''Read and return value from redis_store
        '''
        redis_value = self._redis_config.get_param(self._redis_param_key)
        if redis_value is None and default is None:
            rospy.logwarn(
                "%s:  Unable to initialize offset:  "
                "no saved value and no default given" % self.compname
            )
            return
        new_value = default if redis_value is None else redis_value
        self.set_pin(new_value)
        return new_value

    def set_redis_from_pin(self):
        '''Read HAL pin value and write to redis_store
        '''
        old_val = self._redis_config.get_param(self._redis_param_key)
        new_val = self.get_pin()
        self._redis_config.set_param(self._redis_param_key, new_val)
        if old_val is not None and not self._isclose(
            new_val, old_val, 1e-5, 1e-5
        ):
            rospy.loginfo(
                "%s:  Updated %s from %.4f to %.4f",
                self.compname,
                self._redis_param_key,
                old_val,
                self._redis_config.get_param(self._redis_param_key),
            )
        return new_val

    @classmethod
    def shutdown(cls):
        config = cls._cached_objs.get('redis_config_client', None)
        if config is not None:
            config.stop()
