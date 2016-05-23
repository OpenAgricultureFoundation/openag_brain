from sys import argv
from uuid import uuid4
from importlib import import_module

from couchdb import Server

from .base import ModuleGroup
from .util import get_or_create, update_doc
from .models import ModuleGroupModel
from .db_names import DbName

def register_module_group(cls, db):
    if not issubclass(cls, ModuleGroup):
        raise TypeError()
    cls_path = cls.__module__ + ':' + cls.__name__
    new_info = cls._info._asdict()
    doc = get_or_create(db, cls_path)
    update_doc(db, doc, new_info)

if __name__ == '__main__':
    server = Server()
    db = server[DbName.MODULE_GROUP.value]
    class_path = argv[1]
    package_path, class_name = class_path.split(":")
    package = import_module(package_path)
    class_to_register = getattr(package, class_name)
    register_module_group(class_to_register, db)

