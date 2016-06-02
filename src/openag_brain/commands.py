import os
import json
import shlex
import subprocess

import rospkg
import xml.etree.ElementTree as ET
from xml.dom.minidom import parse
from couchdb import PreconditionFailed

import _design
import fixtures
from .util import get_or_create
from .models import SoftwareModuleTypeModel, SoftwareModuleModel
from .db_names import DbName

def register_modules(server, package_name):
    # Find and parse the modules file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(package_name)
    modules_path = os.path.join(pkg_path, 'modules.xml')
    modules = parse(modules_path)

    # Handle the software modules
    software_modules = modules.getElementsByTagName('software')[0]
    db = server[DbName.SOFTWARE_MODULE_TYPE]
    for module in software_modules.getElementsByTagName('module'):
        module_type = module.attributes["type"].value
        module_id = "{}:{}".format(package_name, module_type)
        module_entry = get_or_create(db, module_id, SoftwareModuleTypeModel)

        description = module.getElementsByTagName('description')[0]
        module_entry.description = description.firstChild.data

        parameters = module.getElementsByTagName('parameter')
        module_entry.parameters = [
            parameter.attributes["name"].value for parameter in parameters
        ]

        inputs = module.getElementsByTagName('input')
        module_entry.inputs = [
            input.attributes["name"].value for input in inputs
        ]

        outputs = module.getElementsByTagName('output')
        module_entry.outputs = [
            output.attributes["name"].value for output in outputs
        ]

        services = modules.getElementsByTagName('service')
        module_entry.services = [
            service.attributes["name"].value for service in services
        ]

        module_entry.store(db)

    # TODO: Handle the firmware modules

def folder_to_dict(path):
    res = {}
    for key in os.listdir(path):
        key_path = os.path.join(path, key)
        if os.path.isfile(key_path):
            val = open(key_path).read()
            key = key.split('.')[0]
            res[key] = val
        else:
            res[key] = folder_to_dict(key_path)
    return res

def init_db(server):
    # Copy the couchdb config file into the correct directory
    config_file_path = os.path.join(os.path.dirname(__file__), "couchdb.ini")
    if os.path.isdir("/etc/couchdb/default.d"):
        dest_path = "/etc/couchdb/default.d/openag.ini"
    elif os.path.isdir("/usr/local/etc/couchdb/default.d"):
        dest_path = "/usr/local/etc/couchdb/default.d/openag.ini"
    else:
        raise RuntimeError("Failed to install couchdb configuration file")
    command = "sudo cp {} {}".format(config_file_path, dest_path)
    print "Running", command
    subprocess.call(shlex.split(command))

    # Create all of the databases
    for k,v in DbName.__dict__.items():
        if k.isupper():
            try:
                server.create(v)
            except PreconditionFailed:
                pass

    # Push design documents to all of the databases
    design_path = os.path.dirname(_design.__file__)
    for db_name in os.listdir(design_path):
        if db_name.startswith('__'):
            continue
        db_path = os.path.join(design_path, db_name)
        db = server[db_name]
        doc = get_or_create(db, "_design/openag")
        doc._data.update(folder_to_dict(db_path))
        doc.store(db)

    register_modules(server, 'openag_brain')

    print "CouchDB may have to be restarted for all changes to take effect"

def load_fixture(server, fixture_name):
    fixture_file_name = os.path.join(
        os.path.dirname(fixtures.__file__), fixture_name + ".json"
    )
    with open(fixture_file_name) as fixture_file:
        fixture = json.load(fixture_file)
    for db_name, items in fixture.items():
        db = server[db_name]
        for item in items:
            item_id = item.pop("_id")
            doc = get_or_create(db, item_id)
            doc._data.update(item)
            doc.store(db)

def update_launch(server):
    db = server[DbName.SOFTWARE_MODULE]

    # Form a launch file from the parameter configuration
    root = ET.Element('launch')
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
