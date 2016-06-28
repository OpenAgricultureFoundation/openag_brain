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
    an `environment`.
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
    or actuator) in the system
    """
    environment = TextField()
    """ (str) The ID of the environment on which this peripheral acts """
    type = TextField()
    """ (str) The ID of the firmware module type of this object """
    parameters = DictField()
    """
    (dict) A dictionary mapping parameter names to parameter values. There
    should be an entry in this dictionary for every `parameter` in the firmware
    module type of this firmware module that doesn't have a default value.
    """
    mappings = DictField()
    """
    (dict) A dictionary mapping ROS names to different ROS names. Keys are the
    names defined in the firmware module type and values are the names that
    should be used instead. This can be used, for example, to route the correct
    input into an actuator module with a generic input name such as `state`.
    """

class FirmwareModuleTypeModel(Document):
    """
    A `firmware module type` represents a firmware library for interfacing with
    a particular system peripheral. It is essentially a driver for a sensor or
    actuator. The code itself should be registered with `platformio
    <platformio.org>`_ and metadata about it should be stored in the OpenAg
    database.
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
    parameters = DictField()
    """
    (dict) A nested dictionary mapping parameter names to dictionaries
    containing information about those parameters. The inner dictionaries must
    contain the field "type" (e.g. "int", "float") and can contain the fields
    "description" and "default". The parameter values will be passed  into the
    constructor of the top-level class of this library.
    """
    inputs = DictField()
    """
    (dict) A nested dictionary mapping names of ROS topics to which this
    library subscribes to dictionaries containing information about those
    topics. The inner dictionary must contain the field "type", which is the
    ROS message type for the topic.
    """
    outputs = DictField()
    """
    (dict) A nested dictionary mapping names of ROS topics to which this
    library publishes to dictionaries containing information about those
    topics. The inner dictionary must contain the field "type", which is the
    ROS message type for the topic.
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
    package = TextField()
    """
    (str) The name of the ROS package containing the code for this object
    """
    executable = TextField()
    """ (str) The name of the executable for this object """
    description = TextField()
    """ (str) Description of the library """
    parameters = DictField()
    """
    (dict) A nested dictionary mapping ROS parameter names required by this
    type of module to dictionaries containing information about those
    parameters. The inner dictionaries must contain the field "type" (e.g.
    "int", "float") and can contain the fields "description" and "default".
    """
    inputs = DictField()
    """
    (dict) A nested dictionary mapping names of ROS topics to which this type
    of module subscribes to dictionaries containing information about those
    topics. The inner dictionary must contain the field "type", which is the
    ROS message type for the topic and can contain the field "description"
    """
    outputs = DictField()
    """
    (dict) A nested dictionary mapping names of ROS topics to which this type
    of module publishes to dictionaries containing information about those
    topics.  The inner dictionary must contain the field "type", which is the
    ROS message type for the topic and can contain the field "description".
    """
    services = DictField()
    """
    (dict) A nested dictionary mapping names of ROS services advertised by modules of
    this type to dictionaries containing information about those services. The
    inner dictionary must contain the field "type", which is the ROS service
    type for the service and can contain the field "description".
    """

