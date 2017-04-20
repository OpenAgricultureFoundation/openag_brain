from nose.tools import assert_equal
from openag_brain.software_modules.pid import PID

def test_p():
    pid = PID(1, 0, 0)
    pid.set_point = 0
    assert_equal(-0.1, pid.update(0.1))
    assert_equal(0.1, pid.update(-0.1))

def test_i():
    pid = PID(0, 1, 0)
    pid.set_point = 0
    assert_equal(-0.1, pid.update(0.1))
    assert_equal(-0.2, pid.update(0.1))
    assert_equal(-0.3, pid.update(0.1))

def test_d():
    pid = PID(0, 0, 1)
    pid.set_point = 0
    assert_equal(-0.1, pid.update(0.1))
    assert_equal(-0, pid.update(0.1))
    assert_equal(0.1, pid.update(0))
