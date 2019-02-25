# -*- coding: utf-8 -*-


"""
   :synopsis: Base class for HAL objects

   .. moduleauthor:: John Morris <john@dovetail-automata.com>
"""

import hal
import rospy


class HalObjBase(object):
    """Base class for HAL component objects

    Takes care of caching objects related to HAL components, like the
    component itself.

    The :py:class:`hal_hw_interface.ros_hal_pin.RosHalComponent` class
    will create the actual HAL component here.  Then pins can access
    the component without it explicitly being passed around.
    """

    _cached_objs = dict()

    def init_hal_comp(self):
        """Initializes a new HAL component

        To be called from
        :py:class:`hal_hw_interface.ros_hal_pin.RosHalComponent`
        object setup to initialize the new HAL component
        """
        if not hasattr(self, 'compname'):
            raise RuntimeError('No "compname" attribute configured')
        if 'hal_comp' in self._cached_objs:
            raise RuntimeError('HAL component already initialized')
        self._cached_objs['hal_comp'] = hal.component(self.compname)

    @property
    def hal_comp(self):
        """Read-only property returning the global HAL component object

        The HAL component must have already been created with
        :py:func:`init_hal_comp`.

        :returns: HAL component object
        :rtype: :py:class:`hal.component`
        """
        if 'hal_comp' not in self._cached_objs:
            raise RuntimeError('No HAL component initialized')
        return self._cached_objs['hal_comp']

    def get_ros_param(self, suffix, default=None):
        '''Retrieve a parameter from the ROS param server, key name prefixed
        with HAL component name

        This is shorthand for retrieving a ROS param server key
        :code:`<compname>/<suffix>`.  The value is cached locally.

        :param suffix: ROS parameter key suffix
        :type suffix: str
        :param default: Default value if key not on param server
        :type default: any
        :returns: parameter value
        :rtype: XmlRpcLegalValue
        '''
        return self._cached_objs.setdefault(
            suffix,
            rospy.get_param('{}/{}'.format(self.compname, suffix), default),
        )
