# -*- coding: utf-8 -*-

# Test keys and values
keys1 = dict(pin1=True, pin2=0.009, pin3=-9)

# More test keys and values, an overlapping set
keys2 = dict(pin1=False, pin2=1.88e42, pin4=0)


class TestFixtures:
    def test_mock_comp_obj_fixture(self, mock_comp_obj, mock_objs):
        # Test hal.component returns mock_comp_obj
        assert mock_objs["hal_comp"]() is mock_comp_obj

        # Set each pin (with out-of-band method) and check
        for name, value in keys1.items():
            mock_comp_obj.set_pin(name, value)
            assert mock_comp_obj[name] == value

        # Recheck
        for name, value in keys1.items():
            assert mock_comp_obj[name] == value

        # Set each pin and check
        for name, value in keys2.items():
            mock_comp_obj[name] = value
            assert mock_comp_obj[name] == value

        # Recheck everything
        pins = keys1.copy()
        pins.update(keys2)
        for name, value in pins.items():
            assert mock_comp_obj[name] == value

        # Default case
        assert mock_comp_obj["bogus"] == 0xDEADBEEF

    def test_mock_rospy_fixture(self, mock_rospy, mock_objs):
        # Test mock rospy.get_param()
        gp = mock_objs["rospy_get_param"]
        gp.set_key("foo", 1)
        gp.set_key("bar", 2)
        assert gp("foo") == 1
        assert gp("bar") == 2
        gp.set_key("baz", 3)
        assert gp("foo") == 1
        assert gp("bar") == 2
        assert gp("baz") == 3

        # Test rospy.Rate() returns expected object
        assert mock_objs["rospy_Rate"]() is mock_objs["rospy_Rate_obj"]

        # Test rospy.is_shutdown() returns True values, then False
        found_false = False
        for i in range(10):
            val = mock_objs["rospy_is_shutdown"]()
            print("iter {} val {}".format(i, val))
            if val is False:
                found_false = True
            if val is True:
                break
        else:
            raise Exception("is_shutdown never returned True")
        if not found_false:
            raise Exception("is_shutdown never returned False")

        # Test returned objects
        for name in ("Subscriber", "Publisher", "Service"):
            method = mock_objs["rospy_{}".format(name)]
            obj = mock_objs["rospy_{}_obj".format(name)]
            assert method() == obj

    def test_mock_redis_client_obj(self, mock_redis_client_obj, mock_objs):
        # Test redis_store.ConfigClient() returns object
        assert mock_objs["redis_store"]() is mock_redis_client_obj

        # Set each param (with out-of-band method) and check
        for name, value in keys1.items():
            mock_redis_client_obj.set_key(name, value)
            assert mock_redis_client_obj.get_param(name) == value

        # Recheck
        for name, value in keys1.items():
            assert mock_redis_client_obj.get_param(name) == value

        # Set each param and check
        for name, value in keys2.items():
            mock_redis_client_obj.set_param(name, value)
            assert mock_redis_client_obj.get_param(name) == value

        # Recheck everything
        params = keys1.copy()
        params.update(keys2)
        for name, value in params.items():
            assert mock_redis_client_obj.get_param(name) == value

        # Default case
        assert mock_redis_client_obj.get_param("bogus") == 0
