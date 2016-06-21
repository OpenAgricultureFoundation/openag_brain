from nose.tools import assert_raises, assert_in
from StringIO import StringIO
from openag_brain.commands.generate_firmware import write_code

class TestType:
    pio_id = 42
    header_file = "test.h"
    class_name = "Test"
    description = "test"
    parameters = ["param1", "param2"]
    inputs = {"test": "std_msgs/Float32"}
    outputs = {"test": "std_msgs/Float32"}

module_types = {
    "test:test": TestType
}


def test_code_gen_valid():
    class TestMod:
        id = "test_1"
        environment = "test_env"
        type = "test:test"
        parameters = {"param1": "test1", "param2": "test2"}
    modules = {"test": TestMod}
    f = StringIO()
    write_code(modules, module_types, f)
    code = f.getvalue()

    assert_in("#include <ros.h>", code)
    assert_in("ros::NodeHandle nh;", code)
    assert_in("nh.initNode()", code)
    assert_in("nh.spinOnce()", code)

    assert_in("#include <test.h>", code)
    assert_in('String test_1_parameters[] = {"test1", "test2"}', code)
    assert_in('Test test_1("test_1", test_1_parameters)', code)
    assert_in('std_msgs::Float32 test_1_test', code)
    assert_in(
        'ros::Publisher pub_test_1_test("/sensors/test_1_test", '
        '&test_1_test_msg)', code
    )
    assert_in('nh.advertise(pub_test_1_test)', code)

def test_code_gen_missing_param():
    class TestMod:
        id = "test_1"
        environment = "test_env"
        type = "test:test"
        parameters = {}
    modules = {"test": TestMod}
    f = StringIO()
    assert_raises(Exception, write_code, modules, module_types, f)
