"""
This module defines a a few simple classes used for describing module
parameters. They just have a type and wrap the parameter description,
allowing ModuleMeta to generate a description of the parameter that includes
both its type and description.
"""
from .server import server

__all__ = [
    'Parameter', 'StringParameter', 'IntegerParameter', 'FloatParameter',
    'ReferenceParameter'
]

class Parameter:
    type = None
    def __init__(self, description):
        self.description = description

    def encode(self, val):
        raise NotImplementedError(
            "Subclasses of `Parameter` should implement `encode`"
        )

class StringParameter(Parameter):
    type = 'string'
    def encode(self, val):
        return str(val)

class IntegerParameter(Parameter):
    type = 'integer'
    def encode(self, val):
        return int(val)

class FloatParameter(Parameter):
    type = 'float'
    def encode(self, val):
        return float(val)

class ReferenceParameter(Parameter):
    def __init__(self, db_name, description):
        self.db_name = db_name
        self.description = description
    def encode(self, val):
        if not val in server[self.db_name]:
            raise ValueError(
                "\"{}\" does not reference a valid object in the \"{}\" "
                "database".format(val, self.db_name)
            )
        return val
