import os
import json
import time
import shlex
import requests
import subprocess
from ConfigParser import ConfigParser

import rospkg
import xml.etree.ElementTree as ET
from xml.dom.minidom import parse
from couchdb import PreconditionFailed
from couchdb.mapping import Document

import _design
import fixtures
from .util import get_or_create, update_doc
from .models import SoftwareModuleTypeModel, SoftwareModuleModel
from .db_names import DbName

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

def init_db(server, hostname):
    # Copy the couchdb config file into the correct directory
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('openag_brain')
    config_file_path = os.path.join(pkg_path, 'couchdb.ini')
    config = ConfigParser()
    config.read(config_file_path)
    for section in config.sections():
        for param, value in config.items(section):
            url = "{}/_config/{}/{}".format(server.resource.url, section, param)
            current_val = requests.get(url).content.strip()
            desired_val = '"{}"'.format(
                value.replace('"', '\\"').replace("$hostname", hostname)
            )
            if current_val != desired_val:
                res = requests.put(
                    url, data=desired_val
                )
                # Unless there is some delay between requests, CouchDB gets sad.
                # I'm not really sure why
                time.sleep(1)
                if not res.status_code == 200:
                    raise RuntimeError(
                        'Failed to set configuration parameter "{}"'.format(param)
                    )

    # Create all of the databases
    for k,v in DbName.__dict__.items():
        if k.isupper():
            try:
                server.create(v)
            except PreconditionFailed:
                # The db already exists
                pass

    # Push design documents to all of the databases
    design_path = os.path.dirname(_design.__file__)
    for db_name in os.listdir(design_path):
        if db_name.startswith('__'):
            continue
        db_path = os.path.join(design_path, db_name)
        db = server[db_name]
        doc = get_or_create(db, "_design/openag")
        update_doc(doc, folder_to_dict(db_path), db)

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
            update_doc(doc, item, db)

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
