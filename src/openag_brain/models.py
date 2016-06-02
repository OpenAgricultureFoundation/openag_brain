from couchdb.mapping import (
    Document, TextField, FloatField, DictField, BooleanField, ListField
)

__all__ = [
    'SoftwareModuleTypeModel', 'SoftwareModuleModel',
    'EnvironmentModel', 'EnvironmentalDataPointModel'
]

class SoftwareModuleTypeModel(Document):
    description = TextField()
    parameters = ListField(TextField())
    inputs = ListField(TextField())
    outputs = ListField(TextField())
    services = ListField(TextField())

class SoftwareModuleModel(Document):
    environment = TextField()
    type = TextField()
    parameters = DictField()
    mappings = DictField()

# TODO: Make FirmwareModuleTypeModel and FirmwareModuleModel

class EnvironmentModel(Document):
    variables = ListField(TextField())

class EnvironmentalDataPointModel(Document):
    environment = TextField()
    variable = TextField()
    is_desired = BooleanField()
    value = TextField()
    timestamp = FloatField()
