from couchdb.mapping import (
    Document, TextField, FloatField, DictField, BooleanField, ListField
)

__all__ = [
    'SoftwareModuleTypeModel', 'SoftwareModuleModel',
    'FirmwareModuleTypeModel', 'FirmwareModuleModel',
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

class FirmwareModuleTypeModel(Document):
    url = TextField()
    header_file = TextField()
    class_name = TextField()
    description = TextField()
    parameters = ListField(TextField())
    inputs = DictField()
    outputs = DictField()

class FirmwareModuleModel(Document):
    environment = TextField()
    type = TextField()
    parameters = DictField()

class EnvironmentModel(Document):
    pass

class EnvironmentalDataPointModel(Document):
    environment = TextField()
    variable = TextField()
    is_desired = BooleanField()
    value = TextField()
    timestamp = FloatField()
