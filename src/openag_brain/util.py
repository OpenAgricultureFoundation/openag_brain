from importlib import import_module
from couchdb.mapping import Document

__all__ = ['get_or_create']

def get_or_create(db, doc_id, Model=Document):
    if doc_id in db:
        return Model.load(db, doc_id)
    else:
        doc = Model(id=doc_id)
        try:
            doc.store(db)
        except Exception:
            # We might not be able to store the empty document because of
            # validation functions in the database. If this happens, it is fine
            # to continue because the document is going to be updated and saved
            # later
            pass
        return doc

def update_doc(doc, updates, db):
    should_save = False
    for k,v in updates.items():
        if k.startswith('_'):
            continue
        if doc.get(k, None) != v:
            should_save = True
            doc[k] = v
    for k,_ in doc.items():
        if k.startswith('_'):
            continue
        if not k in updates:
            should_save = True
            del doc[k]
    if should_save:
        doc.store(db)

def resolve_message_type(msg_type):
    if not msg_type in resolve_message_type.cache:
        pkg, cls = msg_type.split('/')
        mod = import_module('.msg', pkg)
        resolve_message_type.cache[msg_type] = getattr(mod, cls)
    return resolve_message_type.cache[msg_type]
resolve_message_type.cache = {}
