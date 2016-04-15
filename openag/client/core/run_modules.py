#!/usr/bin/env python3

from .base import Module
from .util import get_or_create_db
from .models import ModuleModel, ModuleTypeModel, ModuleConnectionModel
from .db_names import *

import gevent
from couchdb import Server
from importlib import import_module

if __name__ == '__main__':
    server = Server()
    module_db = get_or_create_db(server, MODULE_DB)
    module_type_db = get_or_create_db(server, MODULE_TYPE_DB)
    module_connection_db = get_or_create_db(server, MODULE_CONNECTION_DB)

    # Construct all of the modules
    for mod_id in module_db:
        mod_info = ModuleModel.load(module_db, mod_id)
        mod_type_info = ModuleTypeModel.load(module_type_db, mod_info.type)
        package_path, class_name = mod_type_info.class_path.split(":")
        py_mod = import_module(package_path)
        mod_class = getattr(py_mod, class_name)
        mod = mod_class(mod_id)

    # Initialize all of the modules
    for mod_id in module_db:
        mod_info = ModuleModel.load(module_db, mod_id)
        mod = Module.get_by_id(mod_id)
        mod.init(**mod_info.parameters)

    # Hook up all of the connections
    for mod_conn_id in module_connection_db:
        mod_conn = ModuleConnectionModel.load(module_connection_db, mod_conn_id)
        output_module = Module.get_by_id(mod_conn.output_module)
        input_module = Module.get_by_id(mod_conn.input_module)
        output = getattr(output_module, mod_conn.output_name)
        input = getattr(input_module, mod_conn.input_name)
        output.output_to(input)

    # Run all of the modules
    threads = []
    for mod_id in module_db:
        mod = Module.get_by_id(mod_id)
        mod.start()

    # Listen for changes to the module configuration
    while True:
        gevent.sleep(60)
