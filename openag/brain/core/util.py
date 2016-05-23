import sys
import time
import random
from collections import OrderedDict

from .parameters import *

__all__ = ['get_argument_info', 'gen_doc_id', 'get_or_create', 'update_doc']

def get_argument_info(f):
    arguments = OrderedDict()
    for arg_name in f.__code__.co_varnames[1:f.__code__.co_argcount]:
        annotation = f.__annotations__.get(arg_name, '')
        arg_info = {}
        if isinstance(annotation, Parameter):
            arg_info['type'] = annotation.type
            arg_info['description'] = annotation.description
        else:
            arg_info['description'] = str(annotation)
        arguments[arg_name] = arg_info
    return arguments

def gen_doc_id():
    return "{}-{}".format(time.time(), random.randint(0, sys.maxsize))

def get_or_create(db, doc_id):
    if doc_id in db:
        return db[doc_id]
    else:
        return {"_id": doc_id}

def update_doc(db, doc, updates):
    if any(doc.get(k, None) != v for k,v in updates.items()):
        doc.update(updates)
        db.save(doc)
