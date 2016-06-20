from couchdb.mapping import (
    Document, TextField, FloatField, DictField, BooleanField, ListField
)

__all__ = [
    'EnvironmentModel', 'EnvironmentalDataPointModel', 'FirmwareModuleModel',
    'FirmwareModuleTypeModel', 'RecipeModel', 'SoftwareModuleModel',
    'SoftwareModuleTypeModel']

class EnvironmentModel(Document):
    """
    An `environment` represents a single homogenous controlled environment
    within a system. A food computer usually consists of a single
    `environment`, but larger systems will often contain more than one
    `environment`.

    `Environments` currently have no fields.
    """
    pass

class EnvironmentalDataPointModel(Document):
    """
    An `environmental data point` represents a single measurement or event in
    an `environment.
    """
    environment = TextField()
    """ (str) The ID of the environment for which this point was measured """
    variable = TextField()
    """
    (str) The type of measurement of event this represents (e.g.
    "air_temperature")
    """
    is_desired = BooleanField()
    """
    (bool) This should be true if the data point represents the desired state
    of the environment (e.g. the set points of a recipe) and false if it
    represents the measured state of the environment.
    """
    value = TextField()
    """
    (str) The value associated with the measurement or event. The exact use of
    this field may very depending on the `variable` field.
    """
    timestamp = FloatField()
    """
    (float) A UNIX timestamp reflecting when this data point was generated.
    """

class FirmwareModuleModel(Document):
    """
    A `firmware module` typically drives a single physical peripheral (sensor
    actuator) in the system
    """
    environment = TextField()
    """ (str) The ID of the environment on which this peripheral acts """
    type = TextField()
    """ (str) The ID of the firmware module type of this object """
    parameters = DictField()
    """
    (dict) A dictionary mapping parameter names to parameter values. There
    should be an entry in this dictionary for every `parameter` in the firmware
    module type of this firmware module.
    """

class FirmwareModuleTypeModel(Document):
    """
    A `firmware module type` represents a firmware library for interfacing with
    a particular system peripheral. It is essentially a driver for a sensor or
    actuator. The code itself should be registered with `platformio
    <platformio.org>`_ and metadata about it should be stored in the OpenAg
    database. `Firmware module type`s must define a class that inherits from
    the superclass defined in `openag_peripheral
    <http://github.com/OpenAgInitiative/openag_peripheral>`_, and this object
    must descibe the exact location of that class.
    """
    pio_id = TextField()
    """ (str) The platformio ID of the uploaded library """
    header_file = TextField()
    """
    (str) The name of the header file containing the top-level class in the
    library
    """
    class_name = TextField()
    """ (str) The name of the top-level class in the library """
    description = TextField()
    """ (str) Description of the library """
    parameters = ListField(TextField())
    """
    (array) A list of parameters that must be defined for `firmware module`s of
    this type and passed into the constructor of the top-level class of this
    library
    """
    inputs = DictField()
    """
    (dict) A dictionary mapping names of ROS topics to which this library
    subscribes to ROS message types for those libraries
    """
    outputs = DictField()
    """
    (dict) A dictionary mapping names of ROS topics to which top library
    publishes to ROS message types for those libraries
    """

class RecipeModel(Document):
    """
    In order to allow for recipes to evolve, we have developed a very generic
    recipe model. The idea behind the model is that the system runs a recipe
    handler module which declares some list of recipe formats that it supports.
    Recipes also declare what format they are. Thus, to define a new recipe
    format, you can write a custom recipe handler module type that understands
    that format, write recipes in the new format, and then use the rest of the
    existing system as is.
    """
    format = TextField()
    """ (str) The format of the recipe """
    operations = TextField()
    """
    The actual content of the recipe, organized as specified for the format of
    this recipe
    """

class SoftwareModuleModel(Document):
    """
    A `software module` is an instance of an arbitrary program running on the
    controller. It can listen to ROS topics, publish to ROS topics, and
    advertize services.  Examples include the recipe handler and individual
    control loops.
    """
    environment = TextField()
    """
    (str) The ID of the environment on which this software module acts. Can be
    null
    """
    type = TextField()
    """ (str) The ID of the software module type of this object """
    parameters = DictField()
    """
    (dict) A dictionary mapping parameter names to parameter values. There
    should be an entry in this dictionary for every `parameter` in the spftware
    module type of this software modules
    """
    mappings = DictField()
    """
    (dict) A dictionary mapping ROS names to different ROS names. Keys are the
    names defined in the software module type and values are the names that
    should be used instead. This can be used, for example, to route the correct
    inputs into a control module with generic input names like `set_point` and
    `measured`.
    """

class SoftwareModuleTypeModel(Document):
    """
    A `software module type` is a program that can run on the controller. It
    can listen to ROS topics, publish to ROS topics, and advertize services.
    Examples include the recipe handler and individual control loops. Software
    module types are distributed as ROS packages.
    """
    description = TextField()
    """ (str) Description of the library """
    parameters = ListField(TextField())
    """
    (array) A list of parameters that must be defined for `software modules` of
    this type. They should follow the `ROS naming conventions
    <http://wiki.ros.org/Names>`_.
    """
    inputs = ListField(TextField())
    """
    (dict) A dictionary mapping names of ROS topics to which this library
    subscribes to ROS message types for those libraries
    """
    outputs = ListField(TextField())
    """
    (dict) A dictionary mapping names of ROS topics to which top library
    publishes to ROS message types for those libraries
    """
    services = ListField(TextField())
    """
    (array) A list of names of services advertized by this program
    """

