import sys
import time
import random
from collections import OrderedDict

__all__ = ['gen_doc_id', 'get_or_create', 'update_doc']

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
