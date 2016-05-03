import os
import inspect
from importlib import import_module
from .db_names import DbName
from . import _design
from .. import modules
from .base import ModuleMeta, Module
from .register_module_type import register_module_type
from couchdb import Server, PreconditionFailed

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

if __name__ == '__main__':
    server = Server()

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
        if "_design/openag" in db:
            doc = db["_design/openag"]
        else:
            doc = {"_id": "_design/openag"}
        doc.update(folder_to_dict(db_path))
        server[db_name].save(doc)

    # Create entries in the MODULE_TYPE database for all of the module types
    modules_path = os.path.dirname(modules.__file__)
    for f_name in os.listdir(modules_path):
        if not f_name.endswith('.py'):
            continue
        if f_name.startswith('__'):
            continue
        package_name = '.'.join(__package__.split('.')[:-1])
        py_mod_name = package_name + '.modules.' + f_name.split('.')[0]
        py_mod = import_module(py_mod_name)
        for name, cls in inspect.getmembers(py_mod, inspect.isclass):
            if issubclass(cls, Module) and cls != Module:
                register_module_type(cls, server[DbName.MODULE_TYPE.value])
