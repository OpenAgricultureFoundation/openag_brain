"""
This module defines a a few simple classes used for describing module
parameters. They just have a type and wrap the parameter description,
allowing ModuleMeta to generate a description of the parameter that includes
both its type and description.
"""
__all__ = [
    'Parameter', 'StringParameter', 'IntegerParameter', 'FloatParameter',
    'ReferenceParameter'
]

class Parameter:
    type = None
    def __init__(self, description):
        self.description = description

class StringParameter(Parameter):
    type = 'string'

class IntegerParameter(Parameter):
    type = 'integer'

class FloatParameter(Parameter):
    type = 'float'

class ReferenceParameter(Parameter):
    def __init__(self, db_name, description):
        self.db_name = db_name
        self.description = description
