"""
This module defines a set of classes used to define outups from `Module`s
"""
import time
from .stream import *

__all__ = ['Output']

class OutputChannel:
    """
    Output channels from `Module`s that write data to `Input`s of other
    `Module`s.
    """
    def __init__(self, mod_id, data_type=None, object_id=None):
        self.mod_id = mod_id
        self.data_type = data_type
        self.object_id = object_id
        self.destinations = []

    def output_to(self, dest):
        """
        Tell this channel to write its output to the input given by `dest`
        """
        self.destinations.append(dest)

    def emit(self, value, data_type=None, timestamp=None, object_id=None):
        """
        Sends `item`, which should be a `GeneralStreamItem` instance, to all of
        the destination queues for this channel.
        """
        data_type = data_type or self.data_type
        timestamp = timestamp or time.time()
        object_id = object_id or self.object_id
        if data_type is None:
            raise RuntimeException('No data type specified for output item')
        item = StreamItem(value, data_type, timestamp, self.mod_id, object_id)
        for dest in self.destinations:
            dest.put(item)

class Output:
    """
    A descriptor that creates `OutputChannel` objects for modules instances
    when they are first accessed.
    """
    def __init__(self, data_type=None, object_id=None):
        self.name = None
        self.data_type = data_type
        self.object_id = object_id

    def __get__(self, instance, instance_type):
        if instance is None:
            return self
        setattr(instance, self.name, OutputChannel(
            instance.mod_id, self.data_type, self.object_id
        ))
        return getattr(instance, self.name)
