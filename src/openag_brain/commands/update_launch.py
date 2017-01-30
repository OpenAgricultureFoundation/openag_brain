import os
import rospkg
import lxml.etree as ET

from openag_brain import params
from openag.utils import synthesize_software_module_info
from openag.models import SoftwareModule, SoftwareModuleType
from openag.db_names import SOFTWARE_MODULE, SOFTWARE_MODULE_TYPE
from openag.categories import default_categories, all_categories

from openag_brain.params import CATEGORIES

# maping from python types to roslaunch acceptable ones
PARAM_TYPE_MAPPING = {'float' : 'double'}

def create_node(parent, pkg, type, name, args=None):
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
    if args:
        e.attrib['args'] = args
    return e

def create_param(parent, name, value, type=None):
    """
    Creates an xml node for the launch file that represents a ROS parameter.
    `parent` is the parent xml node. `name` is the name of the parameter to
    set. `value` is the value of the parameter. `type` is the type of the
    paremeter (e.g. int, float)
    """
    e = ET.SubElement(parent, 'param')
    e.attrib['name'] = name
    e.attrib['value'] = value
    if type is not None:
        e.attrib['type'] = PARAM_TYPE_MAPPING.get(type, type)
    return e

def create_list_param(parent, name, value):
    """
    Creates an xml node for the launch file that represents a ROS parameter
    whose value is a list. This function is separate from `create_param`
    because defining as an attribute is invalid in xml, so we have to use the
    `rosparam` tag and insert the list as YAML instead.

    `parent` - The parent xml node for this paramater
    `name` - The name of the parameter to set
    `value` - The value of the parameter
    """
    e = ET.SubElement(parent, 'rosparam')
    e.attrib['param'] = name
    e.text = str(value)
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

def update_launch(server, categories=default_categories):
    """
    Write a roslaunch file to `modules.launch` based on the software module
    configuration read from the `couchdb.Server` instance `server`.
    """
    # Form a launch file from the parameter configuration
    root = ET.Element('launch')
    create_list_param(root, CATEGORIES, categories)

    groups = {None: root}

    module_db = server[SOFTWARE_MODULE]
    module_type_db = server[SOFTWARE_MODULE_TYPE]
    modules = {
        module_id: SoftwareModule(module_db[module_id]) for module_id in
        module_db if not module_id.startswith('_')
    }
    module_types = {
        type_id: SoftwareModuleType(module_type_db[type_id]) for type_id in
        module_type_db if not type_id.startswith('_')
    }
    modules = synthesize_software_module_info(modules, module_types)

    for module_id, module in modules.items():
        print 'Processing module "{}" from server'.format(module_id)

        # Maybe skip this module based on the categories
        for category in module.get("categories", all_categories):
            if category in categories:
                break
        else:
            print 'Skipping module "{}"'.format(module_id)
            continue

        # Put this module in the right namespace
        ns = module.get("namespace")
        environment = module.get("environment")
        mod_ns = (
            ns if ns else
            "environments/{}".format(environment) if environment else
            None
        )
        if not mod_ns in groups:
            group = create_group(root, mod_ns)
            groups[mod_ns] = group
        else:
            group = groups[mod_ns]

        # Load the module type information
        args = module.get("arguments", [])
        args_str = ", ".join(args)
        node = create_node(
            group, module["package"], module["executable"], module_id, args_str
        )
        for param_name, param_info in module["parameters"].items():
            param_type = str(param_info["type"])
            param_val = str(param_info["value"])
            if isinstance(param_info["value"], bool):
                param_val = param_val.lower()
            create_param(node, param_name, param_val, param_type)
        for k,v in module.get("mappings", {}).items():
            create_remap(node, k, v)
    doc = ET.ElementTree(root)

    # Figure out where to write the launch file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('openag_brain')
    launch_path = os.path.join(pkg_path, 'modules.launch')
    doc.write(launch_path, pretty_print=True)
