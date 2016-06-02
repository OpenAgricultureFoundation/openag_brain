from couchdb.mapping import Document

__all__ = ['get_or_create']

def get_or_create(db, doc_id, Model=Document):
    if doc_id in db:
        return Model.load(db, doc_id)
    else:
        return Model(id=doc_id)
