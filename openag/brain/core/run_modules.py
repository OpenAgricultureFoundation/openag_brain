#!/usr/bin/env python3

import logging
import argparse
from uuid import uuid4
from importlib import import_module

import gevent
from flask import Flask, request, jsonify
from flask.ext.cors import CORS
from gevent.wsgi import WSGIServer

from .base import Module, Requester
from .models import ModuleModel, ModuleTypeModel, ModuleConnectionModel
from .db_names import DbName
from .db_server import db_server
from .parameters import Parameter
from .var_types import EnvironmentalVariable

API_VER = '0.0.1'

def main():
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
    module_db = db_server[DbName.MODULE]
    module_type_db = db_server[DbName.MODULE_TYPE]
    module_connection_db = db_server[DbName.MODULE_CONNECTION]
    env_data_db = db_server[DbName.ENVIRONMENTAL_DATA_POINT]

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
        params = mod_info.parameters
        for arg_name in mod.init.__code__.co_varnames[1:mod.init.__code__.co_argcount]:
            annotation = mod.init.__annotations__.get(arg_name, '')
            if isinstance(annotation, Parameter) and arg_name in params:
                params[arg_name] = annotation.encode(params[arg_name])
        mod.init(**mod_info.parameters)

    # Hook up all of the connections
    for mod_conn_id in module_connection_db:
        if mod_conn_id.startswith('_'):
            continue
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
    # Set cross-origin headers
    CORS(app)
    app.debug=True

    mod_id = uuid4().hex
    while Module.get_by_id(mod_id):
        mod_id = uuid4().hex
    app.mod = Module(mod_id)
    app.mod.start()


    def find_recipe_start(env_data_db, env_id):
        """Given a pointer to the environmental data point db, return the
        latest recipe start for a given environment.
        returns a Dict or None.
        """
        # Query latest view with key (requires this design doc to exist)
        recipe_start_view = env_data_db.view('openag/latest',
            key=[env_id, EnvironmentalVariable.RECIPE_START, 'desired'])
        recipe_end_view = env_data_db.view('openag/latest',
            key=[env_id, EnvironmentalVariable.RECIPE_END, 'desired'])
        # Collect results of iterator (ViewResult has no next method).
        recipe_starts = [recipe_start for recipe_start in recipe_start_view]
        recipe_ends = [recipe_end for recipe_end in recipe_end_view]
        # Narrow list of results down to one and unbox the value
        recipe_start = recipe_starts[0].value if len(recipe_starts) else None
        recipe_end = recipe_ends[0].value if len(recipe_ends) else None

        # If we have a recipe_end, check that it is older than the latest
        # recipe_start
        if recipe_start and recipe_end and recipe_start["timestamp"] > recipe_end["timestamp"]:
            return recipe_start
        # If we don't have a recipe end, but do have a recipe_start, return it.
        elif recipe_start:
            return recipe_start
        else:
            return None

    def find_env_recipe_handler_id(module_db, env_id):
        """Find ID of recipe handler that is running in current environment.
        Returns ID or None.
        """
        recipe_type = 'openag.brain.modules.recipe_handler:RecipeHandler'
        module_view = module_db.view('openag/by_type')

        recipe_handler_ids = [
            module.id
            for module in module_view
            if module.key == recipe_type
        ]

        return recipe_handler_ids[0] if len(recipe_handler_ids) > 0 else None


    @app.route("/api/{v}/module/<mod_id>/<endpoint>".format(v=API_VER), methods=['POST'])
    def serve_endpoint(mod_id, endpoint):
        return getattr(app.mod.ask(mod_id), endpoint)(**request.json)


    @app.route("/api/{v}/environment/<env_id>".format(v=API_VER), methods=['GET'])
    def serve_environment(env_id):

        recipe_handler_id = find_env_recipe_handler_id(module_db, env_id)
        recipe_start = find_recipe_start(env_data_db, env_id)
        recipe_id = recipe_start["value"] if recipe_start else None
        recipe_start_timestamp = recipe_start["timestamp"] if recipe_start else None

        return jsonify(
            env_id = env_id,
            recipe_id = recipe_id,
            recipe_start_timestamp = recipe_start_timestamp,
            recipe_handler_id = recipe_handler_id
        )


    http_server = WSGIServer(('', 5000), app)
    logger.info("Listening for requests on http://localhost:5000/")
    http_server.serve_forever()

if __name__ == '__main__':
    main()
