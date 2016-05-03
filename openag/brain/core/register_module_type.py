from .models import ModuleTypeModel
from .db_names import DbName

from sys import argv
from uuid import uuid4
from couchdb import Server
from importlib import import_module

def register_module_type(cls, db):
    package_path = cls.__module__
    cls_name = cls.__name__
    cls_path = ':'.join([package_path, cls_name])
    doc = {}

    # Check if the document already exists
    view = db.view("openag/by_class_path", key=cls_path)
    if len(view):
        # If so, load the old document
        doc_id = view.rows[0].value
        doc = db[doc_id]
    else:
        # Otherwise, we will create the document; generate a unique id for it
        doc_id = uuid4().hex
        while doc_id in db:
            doc_id = uuid4().hex
        doc['_id'] = doc_id
    new_info = cls._info._asdict()
    new_info['package_path'] = package_path
    new_info['class_name'] = cls_name

    # Only apply the update if changes have actually been made to the class
    if any(doc.get(k, None) != v for k,v in new_info.items()):
        doc.update(new_info)
        db.save(doc)

if __name__ == '__main__':
    server = Server()
    db = server[DbName.MODULE_TYPE.value]
    class_path = argv[1]
    package_path, class_name = class_path.split(":")
    package = import_module(package_path)
    class_to_register = getattr(package, class_name)
    register_module_type(class_to_register, db)

