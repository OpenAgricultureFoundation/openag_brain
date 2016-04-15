from .util import get_or_create_db
from .models import ModuleTypeModel
from .db_names import *

import json
from sys import argv
from uuid import uuid4
from couchdb import Server
from importlib import import_module

if __name__ == '__main__':
    server = Server()
    db = get_or_create_db(server, MODULE_TYPE_DB)
    class_path = argv[1]
    package_path, class_name = class_path.split(":")
    package = import_module(package_path)
    class_to_register = getattr(package, class_name)

    doc = {}
    # Check if the document already exists
    view = db.view("openag/by_class_path", key=class_path)
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
    new_info = class_to_register._info._asdict()
    new_info['class_path'] = class_path
    # Only apply the update if changes have actually been made to the class
    if any(doc.get(k, None) != v for k,v in new_info.items()):
        doc.update(new_info)
        db.save(doc)
