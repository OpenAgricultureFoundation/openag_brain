from couchdb.mapping import (
    Document, TextField, FloatField, DictField, BooleanField, ListField
)

__all__ = [
    'ModuleTypeModel', 'ModuleModel', 'ModuleConnectionModel',
    'EnvironmentModel', 'EnvironmentalDataPointModel'
]

class ModuleGroupModel(Document):
    _id = TextField
    description = TextField()
    parameters = DictField()
    inputs = DictField()
    outputs = DictField()
    endpoints = DictField()
    prcedures = DictField()
    groups = ListField(TextField())

class ModuleTypeModel(Document):
    _id = TextField()
    description = TextField()
    parameters = DictField()
    inputs = DictField()
    outputs = DictField()
    endpoints = DictField()
    procedures = DictField()
    groups = ListField(TextField())

class ModuleModel(Document):
    _id = TextField()
    type = TextField()
    parameters = DictField()

class ModuleConnectionModel(Document):
    output_module = TextField()
    output_name = TextField()
    input_module = TextField()
    input_name = TextField()

class EnvironmentModel(Document):
    variables = ListField(TextField())

class EnvironmentalDataPointModel(Document):
    environment = TextField()
    variable = TextField()
    is_desired = BooleanField()
    value = TextField()
    timestamp = FloatField()
