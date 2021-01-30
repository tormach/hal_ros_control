# -*- coding: utf-8 -*-

"""

   :synopsis: HAL pin object connected to `redis_store` parameter
     server for use in :py:mod:`hal_hw_interface.ros_hal_component`

   .. moduleauthor:: John Morris <john@dovetail-automata.com>

   .. inheritance-diagram::
        hal_hw_interface.redis_store_hal_pin.RedisStoreHalPin
"""

import attr
import rospy

# Mock patching breaks with `from redis_store import ConfigClient`; why?
import redis_store.config as redis_config

from hal_hw_interface.ros_hal_pin import RosHalPin, HalPinDir


@attr.s
class RedisStoreHalPin(RosHalPin):
    '''HAL pin attached to :code:`redis_store` ROS package's parameter
    server

    This HAL pin's value may be read from and written to a
    :code:`redis_store` parameter server.  The default redis key name
    is :code:`<compname>/<name>`.

    :param name: The HAL pin name
    :type name: str
    :param key: Set the redis key
    :type key: str
    :param hal_type: HAL pin data type, one of :code:`['BIT', 'U32',
      'S32', 'FLOAT']`
    :type hal_type:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinType`
    :param hal_dir: HAL pin direction, one of :code:`['IN', 'OUT', 'IO']`
    :type hal_dir:  :py:class:`hal_hw_interface.hal_pin_attrs.HalPinDir`
    '''

    key = attr.ib()

    # Attribute default factories
    @key.default
    def _key_default(self):
        return '{}/{}'.format(self.compname, self.pin_name)

    redis_service_timeout_default = 20.0 # seconds to wait for service

    @property
    def _redis_config(self):
        if 'redis_config_client' in self._cached_objs:
            return self._cached_objs['redis_config_client']

        # Autovivify
        timeout = self.get_ros_param(
            'redis_service_timeout',
            self.redis_service_timeout_default
        )
        rospy.loginfo(f"Connecting to redis database, timeout {timeout}s")
        client = redis_config.ConfigClient(subscribe=True)
        client.wait_for_service(timeout=timeout)
        self._cached_objs['redis_config_client'] = client
        # Disconnect from redis at shutdown
        self.add_shutdown_callback(client.stop)

        return client

    def _ros_init(self):
        if self.hal_dir == HalPinDir('IN'):
            # Input pins write value out to redis
            self._prev_pin_val = self.get_pin()
            self._prev_redis_val = None
            rospy.loginfo(f'Updating to redis key "{self.key}"')
        else:
            # Output pins read value from redis; IO pins go both ways
            # but (arbitrarily) take starting value from redis.
            val = self._redis_config.get_param(self.key)
            self._prev_pin_val = self._prev_redis_val = val
            if val is not None:  # Key exists in redis
                self.set_pin(val)
            # Updates from redis are effected through a callback
            self._redis_config.on_update_received.append(self._update_fm_redis)
            rospy.loginfo(f'Updating from redis key "{self.key}"')

    def _update_fm_redis(self, key, value):
        if key != self.key:
            return  # Not applicable
        self.set_pin(value)
        self._prev_pin_val = self._prev_redis_val = value

    def update(self):
        """Write changed pin value to redis for input and IO pins
        """
        new_val = self.get_pin()
        if self.hal_dir == HalPinDir('OUT') or self._prev_pin_val == new_val:
            return  # Not applicable
        self._redis_config.set_param(self.key, new_val)
        self._prev_pin_val = self._prev_redis_val = new_val
