"""
This module defines a set of classes used to define inputs to `Module`s
"""
import gevent
from gevent.queue import Queue

__all__ = ['Input']

class InputQueue:
    """
    Queues stored on `Module` instances that accept data from other modules.
    """
    def __init__(self, data_type=None):
        self.data_type=None
        self.queue = Queue()
        self.callbacks = {}

    def __iter__(self):
        for item in self.queue:
            if self.data_type and item.data_type != self.data_type:
                raise TypeError(
                    'InputQueue received a stream item of the wrong data '
                    'type. Expected {} but got {}'.format(
                        self.data_type, item.data_type
                    )
                )
            yield item

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
        def reader_thread():
            for item in self:
                for callback in self.callbacks.values():
                    callback(item)
        return gevent.spawn(reader_thread)

class Input:
    """
    A descriptor that creates `InputQueue` objects for module instances when
    they are first accessed.
    """
    def __init__(self, data_type=None):
        self.name = None
        self.data_type = data_type

    def __get__(self, instance, instance_type):
        if instance is None:
            return self
        setattr(instance, self.name, InputQueue(self.data_type))
        return getattr(instance, self.name)
