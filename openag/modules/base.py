"""
This module defines the `Module` class (and a couple of helper classes) from
which all modules should inherit.
"""
import gevent
from collections import namedtuple
from .input import Input
from .output import Output

__all__ = ['Module', ]

ModuleInfo = namedtuple('ModuleInfo', ['inputs', 'outputs', 'parameters'])
ModuleInfo.__doc__ = """
Describes a `Module` subclass. `inputs` is a dictionary mapping input names to
data types. `outputs` is a dictionary mapping output names to data types.
`parameters` is a dictionary mapping parameter names to descriptions.
"""

class ModuleMeta(type):
    """
    Metaclass for the Module class. Inspects the class attributes to construct
    a ModuleInfo object describing the Module class
    """
    def __init__(cls, name, bases, attrs):
        inputs = {}
        outputs = {}
        parameters = {}
        for name, attr in attrs.items():
            if isinstance(attr, Input):
                attr.name = name
                inputs[name] = attr.data_type
            elif isinstance(attr, Output):
                attr.name = name
                outputs[name] = attr.data_type
            elif name == 'init':
                # Read the argument names and docstrings from the `init`
                # function
                parameters = {
                    name: attr.__annotations__.get(name, '') for name in
                    attr.__code__.co_varnames[1:attr.__code__.co_argcount]
                }
        info = ModuleInfo(inputs, outputs, parameters)
        cls._info = info

class Module(metaclass=ModuleMeta):
    """
    Parent class for all modules. Subclasses must override `init` and/or `run`.
    """
    def __init__(self, mod_id, **kwargs):
        """
        `mod_id` is the ID of the module in the database
        """
        self.mod_id = mod_id
        self.init(**kwargs)

    def init(self):
        """
        Subclasses of `Module` can override this function to define an
        initialization procedure. Arguments to this function will be
        interpreted as module parameters. There is a default implementation
        that registers callbacks on inputs if it sees a function on the module
        named of the form "on_<input_name>".
        """
        for input_name in self._info.inputs.keys():
            callback_name = "on_{}".format(input_name)
            if hasattr(self, callback_name):
                callback = getattr(self, callback_name)
                getattr(self, input_name).add_callback(callback)

    def run(self):
        """
        This function runs the `Module`. Subclasses of `Module` can override
        this function to define the normal operation of the module. There is a
        default implementation that simply spawns reader threads for each of
        the inputs and waits for them to terminate.
        """
        reader_threads = [
            getattr(self, input).spawn_reader() for input in self._info.inputs
        ]
        gevent.joinall(reader_threads)
