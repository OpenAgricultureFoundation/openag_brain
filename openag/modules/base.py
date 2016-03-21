import gevent
from collections import namedtuple
from .input import Input
from .output import Output

__all__ = ['Module', ]

ModuleInfo = namedtuple('ModuleInfo', ['inputs', 'outputs', 'parameters'])

class ModuleMeta(type):
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
                parameters = {
                    name: attr.__annotations__.get(name, '') for name in
                    attr.__code__.co_varnames[1:attr.__code__.co_argcount]
                }
        info = ModuleInfo(inputs, outputs, parameters)
        cls._info = info

class Module(metaclass=ModuleMeta):
    def __init__(self, mod_id, **kwargs):
        self.mod_id = mod_id
        self.init(**kwargs)

    def init(self):
        pass

    def run(self):
        reader_threads = [
            getattr(self, input).spawn_reader() for input in self._info.inputs
        ]
        gevent.joinall(reader_threads)

    def start(self):
        return gevent.spawn(self.run)
