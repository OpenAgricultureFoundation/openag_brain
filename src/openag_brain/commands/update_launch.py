import os
import rospkg
import lxml.etree as ET

from openag_brain import params
from openag_brain.models import SoftwareModuleModel, SoftwareModuleTypeModel
from openag_brain.db_names import DbName

def create_node(parent, pkg, type, name):
    """
    Creates an xml node for the launch file that represents a ROS node.
    `parent` is the parent xml node. `pkg` is the ROS package of the node.
    `type` is the name of the executable for the node. `name` is the name
    of the ROS node.
    """
    e = ET.SubElement(parent, 'node')
    e.attrib['pkg'] = pkg
    e.attrib['type'] = type
    e.attrib['name'] = name
    return e

def create_param(parent, name, value, type):
    """
    Creates an xml node for the launch file that represents a ROS parameter.
    `parent` is the parent xml node. `name` is the name of the parameter to
    set. `value` is the value of the parameter. `type` is the type of the
    paremeter (e.g. int, float)
    """
    e = ET.SubElement(parent, 'param')
    e.attrib['name'] = name
    e.attrib['value'] = value
    e.attrib['type'] = type
    return e

def create_group(parent, ns):
    """
    Creates an xml node for the launch file that represents a ROS group.
    `parent` is the parent xml node. `ns` is the namespace of the group.
    """
    e = ET.SubElement(parent, 'group')
    e.attrib['ns'] = ns
    return e

def create_remap(parent, from_val, to_val):
    """
    Creates an xml node for the launch file that represents a name remapping.
    `parent` is the parent xml node. `from_val` is the name that is to be
    remapped. `to_val` is the target name.
    """
    e = ET.SubElement(parent, 'remap')
    e.attrib['from'] = from_val
    e.attrib['to'] = to_val

def create_arg(parent, name, default=None, value=None):
    """
    Creates an xml node for the launch file that represents a command line
    argument. `parent` is the parent xml node. `default` is the default value
    of the argument. `value` is the value of the argument. At most one of
    `default` and `value` can be provided.
    """
    e = ET.SubElement(parent, 'arg')
    e.attrib['name'] = name
    if default and value:
        raise ValueError(
            "Argument cannot have both a default value and a value"
        )
    if default is not None:
        e.attrib['default'] = str(default)
    if value is not None:
        e.attrib['value'] = str(value)

def update_launch(server):
    """
    Write a roslaunch file to `modules.launch` based on the software module
    configuration read from the `couchdb.Server` instance `server`.
    """
    module_db = server[DbName.SOFTWARE_MODULE]
    module_type_db = server[DbName.SOFTWARE_MODULE_TYPE]

    # Form a launch file from the parameter configuration
    root = ET.Element('launch')
    create_node(root, 'openag_brain', 'api.py', 'api')
    create_node(root, 'openag_brain', 'topic_connector.py', 'topic_connector')
    create_node(root, 'openag_brain', 'handle_arduino.py', 'handle_arduino')
    create_param(root, params.DB_SERVER, server.resource.url, 'str')
    create_arg(root, params.DEVELOPMENT, default=False)
    create_param(
        root, params.DEVELOPMENT, '$(arg {})'.format(params.DEVELOPMENT), 'str'
    )
    groups = {None: root}
    for module_id in module_db:
        if module_id.startswith('_'):
            continue
        module = SoftwareModuleModel.load(module_db, module_id)
        if not module.environment in groups:
            group = create_group(root, module.environment)
            groups[module.environment] = group
        else:
            group = groups[module.environment]
        module_type = SoftwareModuleTypeModel.load(module_type_db, module.type)
        node = create_node(
            group, module_type.package, module_type.executable, module.id
        )
        for k,v in module.mappings.items():
            create_remap(node, k, v)
        for k,v in module.parameters.items():
            param = ET.SubElement(node, 'param')
            if k.startswith('~'):
                param.attrib['name'] = k[1:]
            else:
                raise RuntimeException()
            param.attrib['value'] = str(v)
            if isinstance(v, int):
                param.attrib['type'] = "int"
            else:
                raise RuntimeException()
    doc = ET.ElementTree(root)

    # Figure out where to write the launch file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('openag_brain')
    launch_path = os.path.join(pkg_path, 'modules.launch')
    doc.write(launch_path, pretty_print=True)
