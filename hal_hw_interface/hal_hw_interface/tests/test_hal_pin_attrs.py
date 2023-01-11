import pytest
import machinekit.hal.pyhal as hal

from hal_hw_interface.hal_pin_attrs import HalPinDir, HalPinType

valid_cases = [
    # Pin direction
    (HalPinDir, "IN", hal.HAL_IN),
    (HalPinDir, "OUT", hal.HAL_OUT),
    (HalPinDir, "IO", hal.HAL_IO),
    # Pin type
    (HalPinType, "BIT", hal.HAL_BIT),
    (HalPinType, "U32", hal.HAL_U32),
    (HalPinType, "S32", hal.HAL_S32),
    (HalPinType, "FLOAT", hal.HAL_FLOAT),
]


@pytest.fixture(params=valid_cases)
def valid_case(request):
    return request.param


def test_hal_pin_attrs_valid_case_fixture(valid_case):
    print(valid_case)
    assert len(valid_case) == 3
    obj_type, short_name, int_val = valid_case
    assert obj_type in (HalPinDir, HalPinType)
    assert short_name in ("IN", "OUT", "IO", "BIT", "U32", "S32", "FLOAT")


def test_hal_pin_attr_new_from_short_name(valid_case):
    # Test init from short name, e.g. 'IN'
    obj_type, short_name, int_val = valid_case
    test_val = obj_type(short_name)
    assert test_val == int_val


def test_hal_pin_attr_new_from_long(valid_case):
    # Test init from long name, e.g. 'HAL_IN'
    obj_type, short_name, int_val = valid_case
    test_val = obj_type("HAL_" + short_name)
    assert test_val == int_val


def test_hal_pin_attr_new_from_hal_enum(valid_case):
    # Test init from integer, e.g. (hal.HAL_IN)
    obj_type, short_name, int_val = valid_case
    test_val = obj_type(int_val)
    assert test_val == int_val


def test_hal_pin_attr_repr(valid_case):
    # Test __repr__ returns e.g. 'HAL_IN'
    obj_type, short_name, int_val = valid_case
    test_val = obj_type(int_val)
    assert repr(test_val) == "HAL_" + short_name


def test_hal_pin_attr_str(valid_case):
    # Test __str__ returns e.g. 'HAL_IN'
    obj_type, short_name, int_val = valid_case
    test_val = obj_type(int_val)
    assert str(test_val) == "HAL_" + short_name


invalid_cases = [
    # Pin direction
    (HalPinDir, "INY"),
    (HalPinDir, "in"),
    (HalPinDir, hal.HAL_BIT),
    (HalPinDir, hal.HAL_U32),
    (HalPinDir, hal.HAL_S32),
    (HalPinDir, hal.HAL_FLOAT),
    (HalPinDir, -1),
    (HalPinDir, None),
    (HalPinDir, float(hal.HAL_IN)),
    # Pin type
    (HalPinType, "BITY"),
    (HalPinType, "bit"),
    (HalPinType, hal.HAL_IN),
    (HalPinType, hal.HAL_OUT),
    (HalPinType, hal.HAL_IO),
    (HalPinType, -1),
    (HalPinType, None),
    (HalPinType, float(hal.HAL_BIT)),
]


@pytest.fixture(params=invalid_cases)
def invalid_case(request):
    return request.param


def test_hal_pin_attrs_invalid_case_fixture(invalid_case):
    print(invalid_case)
    assert len(invalid_case) == 2
    obj_type, bogus_value = invalid_case
    assert obj_type in (HalPinDir, HalPinType)


def test_hal_attr_type_new_invalid(invalid_case):
    # Test init from invalid values raises ValueError
    obj_type, bogus_value = invalid_case
    with pytest.raises(ValueError):
        obj_type(bogus_value)
