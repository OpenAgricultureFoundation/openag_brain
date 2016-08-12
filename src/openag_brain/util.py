import requests
from importlib import import_module

__all__ = ['resolve_message_type']

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

