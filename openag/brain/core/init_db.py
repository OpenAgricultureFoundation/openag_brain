import os
import json
import shutil
import inspect
import argparse
from importlib import import_module

from couchdb import Server, PreconditionFailed

from . import _design, fixtures
from .base import Module, ModuleGroup
from .util import get_or_create, update_doc
from .db_names import DbName
from .register_module_type import register_module_type
from .register_module_group import register_module_group

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

def main():
    parser = argparse.ArgumentParser(description="Initialize the database")
    parser.add_argument('--fixture', required=False)
    args = parser.parse_args()

    server = Server()

    # Copy the couchdb config file into the correct directory
    config_file_path = os.path.join(os.path.dirname(__file__), "couchdb.ini")
    if os.path.isdir("/etc/couchdb/default.d"):
        shutil.copy(config_file_path, "/etc/couchdb/default.d/openag.ini")
    elif os.path.isdir("/usr/local/etc/couchdb/default.d"):
        shutil.copy(
            config_file_path, "/usr/local/etc/couchdb/default.d/openag.ini"
        )
    else:
        raise RuntimeError("Failed to install couchdb configuration file")


    # Create all of the databases
    for db_name in DbName:
        try:
            server.create(db_name.value)
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
        update_doc(db, doc, folder_to_dict(db_path))

    # Create entries in the MODULE_TYPE database for all of the module types
    brain_dir = os.path.dirname(os.path.dirname(__file__))
    mod_dir = os.path.join(brain_dir, 'modules')
    for f_name in os.listdir(mod_dir):
        if not f_name.endswith('.py'):
            continue
        if f_name.startswith('__'):
            continue
        package_name = '.'.join(__package__.split('.')[:-1])
        py_mod_name = package_name + '.modules.' + f_name.split('.')[0]
        py_mod = import_module(py_mod_name)
        for name, cls in inspect.getmembers(py_mod, inspect.isclass):
            if issubclass(cls, Module) and cls.__module__ == py_mod_name:
                register_module_type(cls, server[DbName.MODULE_TYPE.value])

    # Create entries in the MODULE_GROUP database for all of the module groups
    mod_group_dir = os.path.join(brain_dir, 'module_groups')
    for f_name in os.listdir(mod_group_dir):
        if not f_name.endswith('.py'):
            continue
        if f_name.startswith('__'):
            continue
        package_name = '.'.join(__package__.split('.')[:-1])
        py_mod_name = package_name + '.module_groups.' + f_name.split('.')[0]
        py_mod = import_module(py_mod_name)
        for name, cls in inspect.getmembers(py_mod, inspect.isclass):
            if issubclass(cls, ModuleGroup) and cls.__module__ == py_mod_name:
                register_module_group(cls, server[DbName.MODULE_GROUP.value])

    # Create modules based on the fixture passed in from the command line
    if args.fixture:
        fixture_file_name = os.path.join(
            os.path.dirname(fixtures.__file__), args.fixture + ".json"
        )
        with open(fixture_file_name) as fixture_file:
            fixture = json.load(fixture_file)
        for db_name, items in fixture.items():
            db = server[db_name]
            for item in items:
                item_id = item["_id"]
                doc = get_or_create(db, item_id)
                update_doc(db, doc, item)

if __name__ == '__main__':
    main()
