"""
This module defines all of the types of data that can be passed between
modules. There should be one type in this file for each variable type in the
database.
"""
from enum import Enum, unique

__all__ = ['DataType']

@unique
class DataType(Enum):
    TEST = 'test'
    REQUEST = 'request'
    RESPONSE = 'response'
