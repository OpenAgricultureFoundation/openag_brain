from sys import argv
from uuid import uuid4
from importlib import import_module

from .base import ModuleGroup
from .util import get_or_create, update_doc
from .models import ModuleGroupModel
from .db_names import DbName
from .db_server import db_server

def register_module_group(cls, db):
    if not issubclass(cls, ModuleGroup):
        raise TypeError()
    cls_path = cls.__module__ + ':' + cls.__name__
    new_info = cls._info._asdict()
    doc = get_or_create(db, cls_path)
    update_doc(db, doc, new_info)

if __name__ == '__main__':
    db = db_server[DbName.MODULE_GROUP]
    class_path = argv[1]
    package_path, class_name = class_path.split(":")
    package = import_module(package_path)
    class_to_register = getattr(package, class_name)
    register_module_group(class_to_register, db)

