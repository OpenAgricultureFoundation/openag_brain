from .stream import GeneralStreamItem as StreamItem

__all__ = ['GeneralOutput', 'SimpleOutput']

class OutputChannel:
    def __init__(self, mod_id):
        self.mod_id = mod_id
        self.destinations = []

    def output_to(self, dest):
        self.destinations.append(dest)

    def emitItem(self, item):
        for dest in self.destinations:
            dest.put(item)

class GeneralOutputChannel(OutputChannel):
    def emit(self, data_type, value):
        self.emitItem(StreamItem(self.mod_id, data_type, value))

class SimpleOutputChannel(OutputChannel):
    def __init__(self, mod_id, data_type):
        super().__init__(mod_id)
        self.data_type = data_type

    def emit(self, value):
        self.emitItem(StreamItem(self.mod_id, self.data_type, value))

class Output:
    def __init__(self, data_type=None):
        self.name = None
        self.data_type = data_type

    def build_channel(self, mod_id):
        raise NotImplementedError()

    def __get__(self, instance, instance_type):
        if instance is None:
            return self
        setattr(instance, self.name, self.build_channel(instance.mod_id))
        return getattr(instance, self.name)

class GeneralOutput(Output):
    def build_channel(self, mod_id):
        return GeneralOutputChannel(mod_id)

class SimpleOutput(Output):
    def build_channel(self, mod_id):
        return SimpleOutputChannel(mod_id, self.data_type)
