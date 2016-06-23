import os
import requests
from importlib import import_module
from couchdb.mapping import Document

__all__ = [
    'get_or_create', 'update_doc', 'resolve_message_type',
    'get_database_changes', 'read_module_data'
]

def get_or_create(db, doc_id, Model=Document):
    """
    Attempts to retrieve an instance of the `Model` document from the
    `couchdb.client.Database` instance `db` with ID `doc_id`. Creates the
    document in the database if it doesn't already exist.
    """
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
    """
    Updates the document `doc` to contain only the data in `updates` using the
    `couchdb.client.Database` instance `db` as the storage backend.
    """
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
    """
    Resolves a string containing a ROS message type (e.g. "std_msgs/Float32")
    to the Python class for that message type
    """
    if not msg_type in resolve_message_type.cache:
        pkg, cls = msg_type.split('/')
        mod = import_module('.msg', pkg)
        resolve_message_type.cache[msg_type] = getattr(mod, cls)
    return resolve_message_type.cache[msg_type]
resolve_message_type.cache = {}

def get_database_changes(server_url, db_name, last_seq=None):
    """
    Queries the change feed on the server at `server_url` for the database
    given by `db_name`. Optional parameter `last_seq` is the sequence number of
    the last update that has already been processed by the client. Returns a
    dictionary containing a new 'last_seq' and a list of 'results'
    """
    query_url = server_url + "/{}/_changes".format(db_name)
    if last_seq:
        query_url += "?last-event-id={}".format(last_seq)
    return requests.get(query_url).json()

def pio_build_path():
    """
    Pio projects are currently built in `~/.openag/build`. This function
    creates that directory if it doesn't exists already and returns the path to
    it.
    """
    home = os.path.expanduser("~")
    openag_folder = os.path.join(home, ".openag")
    if not os.path.isdir(openag_folder):
        os.mkdir(openag_folder)
    build_folder = os.path.join(openag_folder, "build")
    if not os.path.isdir(build_folder):
        os.mkdir(build_folder)
    return build_folder

def read_module_data(module_db, module_cls, module_type_db, module_type_cls):
    """
    Pulls all of modules from `module_db` and all of the module types for which
    there exists modules from `module_type_db`. Assumes modules have a `type`
    attribute that stores the ID of their module type. Returns one dictionary
    mapping module IDs to `module_cls` instances and one dictionary mapping
    module type IDs to `module_type_cls` instances. This function is used in
    generating both the roslaunch file and the firmware code.
    """
    modules = {
        module_id: module_cls.load(module_db, module_id) for module_id in
        module_db if not module_id.startswith('_')
    }
    module_types = {}
    for module in modules.values():
        if not module.type in module_types:
            module_types[module.type] = module_type_cls.load(
                module_type_db, module.type
            )
        if module_types[module.type] is None:
            raise ValueError(
                'Module "{}" references nonexistant module type "{}"'.format(
                    module.id, module.type
                )
            )
    return modules, module_types

