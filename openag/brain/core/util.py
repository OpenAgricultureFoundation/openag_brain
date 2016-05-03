import sys
import time
import random
from collections import OrderedDict
from .parameters import *

__all__ = ['get_argument_info', 'gen_doc_id']

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
