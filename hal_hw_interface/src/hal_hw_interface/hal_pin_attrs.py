# -*- coding: utf-8 -*-
import machinekit.hal.pyhal as hal


class HalPinAttrBase(int):
    """Subclass int to make a simple interface for accessing
    hal.HAL_<suffix> enums by integer or string
    """

    _suffixes = []  # hal.HAL_%s; define in subclasses
    _updated = False

    # Maps for translating constructor input and human-readable output
    _fwd_map = dict()  # 16 -> 'IO'
    _bwd_map = dict()  # 'HAL_IN' -> 16; 'IN' -> 16
    for attr in dir(hal):
        if not attr.startswith("HAL_"):
            continue
        value = getattr(hal, attr)
        attr_short = attr[4:]
        _fwd_map[value] = attr_short
        _bwd_map[attr_short] = value

    def __new__(cls, value):
        """Create new object, translating strings to ints and validating
        value"""
        if isinstance(value, int):
            if cls._fwd_map.get(value, None) not in cls._suffixes:
                raise ValueError("Illegal value '{}'".format(value))
            return int.__new__(cls, value)
        elif isinstance(value, str):
            if value.startswith("HAL_"):
                value = value[4:]
            if value not in cls._suffixes:
                raise ValueError("Illegal value '{}'".format(value))
            return int.__new__(cls, cls._bwd_map[value])
        else:
            raise ValueError("Illegal value '{}'".format(value))

    def __repr__(self):
        return "HAL_" + self._fwd_map[self]

    def __str__(self):
        return self.__repr__()


class HalPinDir(HalPinAttrBase):
    """A HAL pin direction

    This :py:class:`int` type comes from the :py:mod:`HAL` module
    :code:`HAL_IN`, :code:`HAL_OUT`, :code:`HAL_IO` attributes.  These
    may be specified in the constructor as the short or long string,
    e.g. :code:`'OUT'` or :code:`'HAL_IO'`.

    .. inheritance-diagram:: hal_hw_interface.hal_pin_attrs.HalPinDir
    """

    _suffixes = set(["IN", "OUT", "IO"])


class HalPinType(HalPinAttrBase):
    """A HAL pin type

    This :py:class:`int` type comes from the :py:mod:`HAL` module
    :code:`HAL_BIT`, :code:`HAL_U32`, :code:`HAL_S32`,
    :code:`HAL_FLOAT` attributes.  These may be specified in the
    constructor as the short or long string, e.g. :code:`'BIT'` or
    :code:`'HAL_FLOAT'`.

    .. inheritance-diagram:: hal_hw_interface.hal_pin_attrs.HalPinType
    """

    _suffixes = set(["BIT", "U32", "S32", "FLOAT"])
