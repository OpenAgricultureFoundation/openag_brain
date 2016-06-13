import os
import rospkg
import xml.etree.ElementTree as ET

from openag_brain.models import SoftwareModuleModel
from openag_brain.db_names import DbName

def update_launch(server):
    db = server[DbName.SOFTWARE_MODULE]

    # Form a launch file from the parameter configuration
    root = ET.Element('launch')
    api = ET.SubElement(root, 'node')
    api.attrib['pkg'] = 'openag_brain'
    api.attrib['type'] = 'api.py'
    api.attrib['name'] = 'api'
    ft = ET.SubElement(root, 'node')
    ft.attrib['pkg'] = 'openag_brain'
    ft.attrib['type'] = 'topic_connector.py'
    ft.attrib['name'] = 'topic_connector'
    groups = {None: root}
    for module_id in db:
        if module_id.startswith('_'):
            continue
        module = SoftwareModuleModel.load(db, module_id)
        if not module.environment in groups:
            group = ET.SubElement(root, 'group')
            group.attrib['ns'] = module.environment
        else:
            group = groups[module.environment]
        node_pkg, node_name = module.type.split(':')
        node = ET.SubElement(group, 'node')
        node.attrib['pkg'] = node_pkg
        node.attrib['type'] = node_name
        node.attrib['name'] = module.id
        for k,v in module.mappings.items():
            remap = ET.SubElement(node, 'remap')
            remap.attrib['from'] = k
            remap.attrib['to'] = v
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
    doc.write(launch_path)
