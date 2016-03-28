"""
This module defines a set of classes used to define outputs from `Module`s.
Modules put an item to an `OutputChannel` stored on the module instance, and
the `OutputChannel` handles converting the item to a `GeneralStreamItem` if
necessary and writing it to the set of modules to which it was told to direct
output.
"""
from .stream import GeneralStreamItem as StreamItem

__all__ = ['GeneralOutput', 'SimpleOutput']

class OutputChannel:
    """
    Abstract base class for output channels from `Module`s. Subclasses of
    `OutputChannel` should define an `emit` method that the `Module` uses to
    write items to the channel.
    """
    def __init__(self, mod_id):
        self.mod_id = mod_id
        self.destinations = []

    def output_to(self, dest):
        """
        Tell this channel to write its output to the input given by `dest`
        """
        self.destinations.append(dest)

    def emitItem(self, item):
        """
        Sends `item`, which should be a `GeneralStreamItem` instance, to all of
        the destination queues for this channel.
        """
        for dest in self.destinations:
            dest.put(item)

class GeneralOutputChannel(OutputChannel):
    """
    Subclass of `OutputChannel`. Requires users to specify the data type of
    every value they output
    """
    def emit(self, data_type, value):
        self.emitItem(StreamItem(self.mod_id, data_type, value))

class SimpleOutputChannel(OutputChannel):
    """
    Subclass of `OutputChannel` The data type is specified at initialization,
    and then to emit a new stream item, users simply specify the value to
    output.
    """
    def __init__(self, mod_id, data_type):
        super().__init__(mod_id)
        self.data_type = data_type

    def emit(self, value):
        self.emitItem(StreamItem(self.mod_id, self.data_type, value))

class Output:
    """
    Abstract base class for module outputs. It is just a descriptor that
    creates `OutputChannel` objects for module instances when they are first
    accessed.
    """
    def __init__(self, data_type=None):
        self.name = None
        self.data_type = data_type

    def build_channel(self, mod_id):
        """
        Build an `OutputChannel` object to store on a module instance.
        Subclasses of `Output` must override this method.
        """
        raise NotImplementedError()

    def __get__(self, instance, instance_type):
        if instance is None:
            return self
        setattr(instance, self.name, self.build_channel(instance.mod_id))
        return getattr(instance, self.name)

class GeneralOutput(Output):
    """ `Output` subclass that uses `GeneralOutputChannel`s """
    def build_channel(self, mod_id):
        return GeneralOutputChannel(mod_id)

class SimpleOutput(Output):
    """ `Output` subclass that uses `SimpleOutputChannel`s """
    def build_channel(self, mod_id):
        return SimpleOutputChannel(mod_id, self.data_type)
