#!/usr/bin/env python3

import logging
import argparse
from uuid import uuid4
from importlib import import_module

import gevent
from flask import Flask, request
from couchdb import Server
from gevent.wsgi import WSGIServer

from .base import Module, Requester
from .models import ModuleModel, ModuleTypeModel, ModuleConnectionModel
from .db_names import DbName

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run all modules defined for '
    'this system')
    parser.add_argument('--verbose', '-v', action='count', default=0)
    parser.add_argument('--log', default=None)
    args = parser.parse_args()

    # Configure logging
    logger = logging.getLogger("openag_brain")
    logger.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        "%(levelname)s %(asctime)s (%(name)s): %(message)s"
    )
    stream_log_level = max(10, 30-10*args.verbose)
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    stream_handler.setLevel(stream_log_level)
    logger.addHandler(stream_handler)
    if args.log:
        file_handler = logging.FileHandler(args.log)
        file_handler.setFormatter(formatter)
        file_handler.setLevel(min(logging.INFO, stream_log_level))
        logger.addHandler(file_handler)

    # Connect to the databases
    server = Server()
    module_db = server[DbName.MODULE.value]
    module_type_db = server[DbName.MODULE_TYPE.value]
    module_connection_db = server[DbName.MODULE_CONNECTION.value]

    # Construct all of the modules
    for mod_id in module_db:
        if mod_id.startswith('_'):
            continue
        mod_info = ModuleModel.load(module_db, mod_id)
        mod_type_info = ModuleTypeModel.load(module_type_db, mod_info.type)
        package_path, class_name = mod_type_info._id.split(':')
        py_mod = import_module(package_path)
        mod_class = getattr(py_mod, class_name)
        mod = mod_class(mod_id)

    # Initialize all of the modules
    for mod_id in module_db:
        if mod_id.startswith('_'):
            continue
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
        if mod_id.startswith('_'):
            continue
        mod = Module.get_by_id(mod_id)
        mod.start()
    logger.info("Running {} modules".format(len(module_db)))

    # Create and run a Flask app for calling endpoints
    app = Flask(__name__)
    app.debug=True

    mod_id = uuid4().hex
    while Module.get_by_id(mod_id):
        mod_id = uuid4().hex
    app.mod = Module(mod_id)
    app.mod.start()

    @app.route("/<mod_id>/<endpoint>", methods=['POST'])
    def serve_endpoint(mod_id, endpoint):
        return getattr(app.mod.ask(mod_id), endpoint)(**request.json)

    http_server = WSGIServer(('', 5000), app)
    logger.info("Listening for requests on http://localhost:5000/")
    http_server.serve_forever()
