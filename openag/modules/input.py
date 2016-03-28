"""
This module defines a set of classes used to define inputs to `Module`s. Inputs
always operate on streams of `GeneralStreamItem`s internally but can give the
`Module` a different view of the stream for convenience.
"""
import gevent
from gevent.queue import Queue
from .stream import *

__all__ = [
    'GeneralInput', 'TypedInput', 'SourcedInput', 'SimpleInput'
]

class InputQueue:
    """
    Abstract base class for queues stored on `Module` instances that accept
    data from other modules. Subclasses of `InputQueue` must define an
    `__iter__` method that allows `Module`s to iterate through the items in the
    queue.
    """
    def __init__(self):
        self.queue = Queue()
        self.callbacks = {}

    def add_callback(self, callback):
        """
        Register a callback function `callback` that will be called with every
        item received by this queue as soon as it is received if there exists a
        reader thread for this queue.
        """
        self.callbacks[callback.__name__] = callback

    def put(self, item):
        """
        Add `item` to this queue.
        """
        self.queue.put(item)

    def spawn_reader(self):
        """
        Spawns and returns a handle to a thread that reads from this queue and
        calls all of the registered callbacks for each item encountered.
        """
        if len(self.callbacks) == 0:
            return None
        def reader_thread():
            for item in self:
                for callback in self.callbacks.values():
                    callback(item)
        return gevent.spawn(reader_thread)

class GeneralInputQueue(InputQueue):
    """
    A "raw" `InputQueue` subclass. Iterating through it yields the
    `GeneralStreamItem`s that are stored in the queue internally.
    """
    def __iter__(self):
        yield from self.queue

class TypedInputQueue(InputQueue):
    """
    Subclass of `InputQueue`. Converts the data stream to a stream of
    `TypedStreamItem`s.
    """
    def __iter__(self):
        for item in self.queue:
            yield TypedStreamItem(item.data_type, item.value)

class SourcedInputQueue(InputQueue):
    """
    Subclass of `InputQueue`. Converts the data stream to a stream of
    `SourcedStreamItem`s. Also, checks that all received items are of the
    correct data type.
    """
    def __init__(self, data_type=None):
        self.data_type = data_type
        super().__init__()

    def __iter__(self):
        for item in self.queue:
            if self.data_type and item.data_type != self.data_type:
                raise TypeError()
            yield SourcedStreamItem(item.src_id, item.value)

class SimpleInputQueue(InputQueue):
    """
    Subclass of `InputQueue`. Looks like a stream of data values not
    encapsulated in stream item objects. If you don't know which queue type you
    want, you probably want this one. Also, checks that all received items are
    of the correct data type.
    """
    def __init__(self, data_type):
        self.data_type = data_type
        super().__init__()

    def __iter__(self):
        for item in self.queue:
            if self.data_type and item.data_type != self.data_type:
                raise TypeError()
            yield item.value

class Input:
    """
    Abstract base class for module inputs. It is just a descriptor that creates
    `InputQueue` objects for module instances when they are first accessed.
    """
    def __init__(self, data_type=None):
        self.name = None
        self.data_type = data_type

    def build_queue(self):
        """
        Build an `InputQueue` object to store on a module instance. Subclasses
        of `Input` must override this method.
        """
        raise NotImplementedError()

    def __get__(self, instance, instance_type):
        if instance is None:
            return self
        setattr(instance, self.name, self.build_queue())
        return getattr(instance, self.name)

class GeneralInput(Input):
    """ `Input` subclass that uses `GeneralInputQueue`s """
    def build_queue(self):
        return GeneralInputQueue()

class TypedInput(Input):
    """ `Input` subclass that uses `TypedInputQueue`s """
    def build_queue(self):
        return TypedInputQueue()

class SourcedInput(Input):
    """ `Input` subclass that uses `SourcedInputQueue`s """
    def build_Queue(self):
        return SourcedInputQueue(self.data_type)

class SimpleInput(Input):
    """ `Input` subclass that uses `SimpleInputQueue`s """
    def build_queue(self):
        return SimpleInputQueue(self.data_type)
