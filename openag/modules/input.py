import gevent
from gevent.queue import Queue
from .stream import *

__all__ = [
    'GeneralInput', 'TypedInput', 'SourcedInput', 'SimpleInput'
]

class InputQueue:
    def __init__(self):
        self.queue = Queue()
        self.callbacks = []

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def put(self, item):
        self.queue.put(item)

    def spawn_reader(self):
        if len(self.callbacks) == 0:
            return None
        def reader_thread():
            for item in self:
                for callback in self.callbacks:
                    callback(item)
        return gevent.spawn(reader_thread)

class GeneralInputQueue(InputQueue):
    def __iter__(self):
        yield from self.queue

class TypedInputQueue(InputQueue):
    def __iter__(self):
        for item in self.queue:
            yield TypedStreamItem(item.data_type, item.value)

class SourcedInputQueue(InputQueue):
    def __init__(self, data_type=None):
        self.data_type = data_type
        super().__init__()

    def __iter__(self):
        for item in self.queue:
            if self.data_type and item.data_type != self.data_type:
                raise TypeError()
            yield SourcedStreamItem(item.src_id, item.value)

class SimpleInputQueue(InputQueue):
    def __init__(self, data_type):
        self.data_type = data_type
        super().__init__()

    def __iter__(self):
        for item in self.queue:
            if self.data_type and item.data_type != self.data_type:
                raise TypeError()
            yield item.value

class Input:
    def __init__(self, data_type=None):
        self.name = None
        self.data_type = data_type

    def build_queue(self):
        raise NotImplementedError()

    def __get__(self, instance, instance_type):
        if instance is None:
            return self
        setattr(instance, self.name, self.build_queue())
        return getattr(instance, self.name)

class GeneralInput(Input):
    def build_queue(self):
        return GeneralInputQueue()

class TypedInput(Input):
    def build_queue(self):
        return TypedInputQueue()

class SourcedInput(Input):
    def build_Queue(self):
        return SourcedInputQueue(self.data_type)

class SimpleInput(Input):
    def build_queue(self):
        return SimpleInputQueue(self.data_type)
