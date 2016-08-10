import requests
from importlib import import_module

__all__ = ['get_database_changes']

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

