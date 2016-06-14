import os
import rospkg
import lxml.etree as ET

from openag_brain.models import SoftwareModuleModel
from openag_brain.db_names import DbName

def create_node(parent, pkg, type, name):
    e = ET.SubElement(parent, 'node')
    e.attrib['pkg'] = pkg
    e.attrib['type'] = type
    e.attrib['name'] = name
    return e

def create_param(parent, name, value, type):
    e = ET.SubElement(parent, 'param')
    e.attrib['name'] = name
    e.attrib['value'] = value
    e.attrib['type'] = type
    return e

def create_group(parent, ns):
    e = ET.SubElement(parent, 'group')
    e.attrib['ns'] = ns
    return e

def create_remap(parent, from_val, to_val):
    e = ET.SubElement(parent, 'remap')
    e.attrib['from'] = from_val
    e.attrib['to'] = to_val

def update_launch(server):
    db = server[DbName.SOFTWARE_MODULE]

    # Form a launch file from the parameter configuration
    root = ET.Element('launch')
    create_node(root, 'openag_brain', 'api.py', 'api')
    create_node(root, 'openag_brain', 'topic_connector.py', 'topic_connector')
    create_param(root, 'database', server.resource.url, "str")
    groups = {None: root}
    for module_id in db:
        if module_id.startswith('_'):
            continue
        module = SoftwareModuleModel.load(db, module_id)
        if not module.environment in groups:
            group = create_group(root, module.environment)
        else:
            group = groups[module.environment]
        node_pkg, node_name = module.type.split(':')
        node = create_node(group, node_pkg, node_name, module.id)
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
