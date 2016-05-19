"""
This module defines a set of classes used to define inputs to `Module`s
"""
import gevent
import logging
from gevent.queue import Queue

__all__ = ['Input']

class InputQueue:
    """
    Queues stored on `Module` instances that accept data from other modules.
    """
    def __init__(self, mod_id, name, data_type=None):
        self.mod_id = mod_id
        self.name = name
        self.data_type=None
        self.queue = Queue()
        self.callbacks = {}
        self._logger = logging.getLogger(
            "openag_brain.module.{}.{}".format(mod_id, name)
        )

    def __iter__(self):
        for item in self.queue:
            self._logger.debug("Received {}".format(item))
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

    def read(self):
        """
        Reads from this queue and calls all of the registered callbacks for
        each item encountered
        """
        for item in self:
            for callback in self.callbacks.values():
                callback(item)

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
        setattr(instance, self.name, InputQueue(
            instance.mod_id, self.name, self.data_type
        ))
        return getattr(instance, self.name)
