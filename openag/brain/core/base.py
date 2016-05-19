"""
This module defines the `Module` class from which all modules should inherit
and some helper classes used to simplify interaction with modules
"""
import time
import gevent
import logging
from gevent.event import AsyncResult
from collections import namedtuple
from .endpoints import *
from .input import *
from .output import *
from .stream import *
from .util import *
from .var_types import *

__all__ = ['Module', ]

ModuleInfo = namedtuple(
    'ModuleInfo', ['inputs', 'outputs', 'parameters', 'endpoints', 'procedures']
)
ModuleInfo.__doc__ = """
Describes a `Module` subclass. `inputs` is a dictionary mapping input names to
data types. `outputs` is a dictionary mapping output names to data types.
`parameters` is a dictionary mapping parameter names to info dictionaries
with a `description` and optionally a `type`. `endpoints` is a dictionary
mapping endpoint names to info dictionaries with a `description` and an
`arguments` dictionary with the same structure as the `parameters`
dictionary. `procedures` is a dictionary mapping procedure names to info
dictionaries with a `description` and a list `endpoints` of names of endpoints.
"""

class ModuleMeta(type):
    """
    Metaclass for the Module class. Inspects the class attributes to construct
    a ModuleInfo object describing the Module class
    """
    def __init__(cls, name, bases, attrs):
        inputs = {}
        outputs = {}
        endpoints = {}
        procedures = {}
        for name, attr in attrs.items():
            # Handle inputs and outputs
            if isinstance(attr, Input):
                attr.name = name
                # Ignore private inputs
                if not name.startswith('_'):
                    inputs[name] = getattr(attr.data_type, 'value', None)
                continue
            elif isinstance(attr, Output):
                attr.name = name
                # Ignore private outputs
                if not name.startswith('_'):
                    outputs[name] = getattr(attr.data_type, 'value', None)
                continue

            # Handle endpoints
            if hasattr(attr, '__call__'):
                if getattr(attr, 'is_endpoint', False):
                    info = {
                        'description': attr.__doc__,
                        'arguments': get_argument_info(attr)
                    }
                    endpoints[name] = info
                    continue

            # Handle procedures
            if isinstance(attr, Procedure):
                procedures[name] = attr.to_dict()

        # Read the argument names and docstrings from the `init` function
        init = attrs.get('init', None)
        parameters = get_argument_info(init) if init else {}

        # Finally construct the `ModuleInfo` object
        info = ModuleInfo(inputs, outputs, parameters, endpoints, procedures)
        cls._info = info

class AsyncRequest:
    """
    Class that is used for sending requests to modules. It represents a
    function that can be called on the target module. Calling it sends a
    request to the target module with the function name and whatever
    arguments were supplied and returns a AsyncResult that will be populated
    with the result of the request
    """
    def __init__(self, src_mod, dest_mod, fn_name):
        self.src_mod = src_mod
        self.dest_mod = dest_mod
        self.fn_name = fn_name

    def __call__(self, *args, **kwargs):
        result = AsyncResult()
        handle = self.src_mod.get_request_handle()
        self.src_mod._open_requests[handle] = result
        item = StreamItem(
            (self.fn_name, args, kwargs), InternalVariable.REQUEST,
            time.time(), self.src_mod.mod_id, handle
        )
        self.dest_mod._requests.put(item)
        return result

class Request(AsyncRequest):
    """
    The same as AsyncRequest except that it blocks and returns the result of
    the request instead of returning an AsyncResult
    """
    def __call__(self, *args, **kwargs):
        return super().__call__(*args, **kwargs).get()

class AsyncRequester:
    """
    Class that is used for sending requests to modules. The `Module.ask`
    function returns an instance of this class, and the user can call a
    function on the instance to send a request to the target module.
    Internally, accessing an attribute on the instance returns a `AsyncRequest`
    object representing the requested function.
    """
    def __init__(self, src_mod, dest_mod):
        self.src_mod = src_mod
        self.dest_mod = dest_mod

    def __getattr__(self, name):
        return AsyncRequest(self.src_mod, self.dest_mod, name)

class Requester:
    """
    The same as AsyncRequester except that it returns a `Request` instead of
    an `AsyncRequest`.
    """
    def __init__(self, src_mod, dest_mod):
        self.src_mod = src_mod
        self.dest_mod = dest_mod

    def __getattr__(self, name):
        return Request(self.src_mod, self.dest_mod, name)

class Module(metaclass=ModuleMeta):
    """
    Parent class for all modules. Subclasses must override `init` and/or
    `run` to define the module's operation.
    """
    _requests = Input(InternalVariable.REQUEST)
    _responses = Input(InternalVariable.RESPONSE)

    def __init__(self, mod_id):
        """
        `mod_id` is the ID of the module in the database
        """
        self.mod_id = mod_id
        if mod_id in Module._registry:
            raise ValueError('A module with this id has already been created')
        Module._registry[mod_id] = self

        # Used internally for handling requests
        self._next_request_handle = 0
        self._open_requests = {}

        # For each input named <input>, if there is a function on the class
        # named 'on_<input>', register it as a callback on the input
        for input_name in self._info.inputs.keys():
            callback_name = "on_{}".format(input_name)
            if hasattr(self, callback_name):
                callback = getattr(self, callback_name)
                getattr(self, input_name).add_callback(callback)

        # Explicitly register callbacks for the private inputs because they
        # don't show up in _info.inputs
        self._requests.add_callback(self.on_request)
        self._responses.add_callback(self.on_response)

        self._threads_to_spawn = []
        self._threads = []
        self.is_running = False

        # Spawn reader threads for all of the inputs
        for input in self._info.inputs:
            self.spawn(getattr(self, input).read)
        self.spawn(self._requests.read)
        self.spawn(self._responses.read)

        # Create a logger for this module
        self._logger = logging.getLogger(
            "openag_brain.module.{}".format(mod_id)
        )

    @classmethod
    def get_by_id(cls, mod_id):
        """ Returns the module with the id `mod_id` """
        return cls._registry.get(mod_id, None)

    def init(self, *args, **kwargs):
        """
        Subclasses of `Module` can override this function to define an
        initialization procedure. Arguments to this function will be
        interpreted as module parameters.
        """
        pass

    def spawn(self, f):
        """
        If this module has already been started, then this spawns a thread that
        runs the given function. Otherwise, it adds the function to a list of
        functions to spawn threads for when the modules is started.
        """
        if self.is_running:
            self._threads.append(gevent.spawn(f))
        else:
            self._threads_to_spawn.append(f)

    def start(self):
        """ Starts this module.  """
        self.is_running = True
        for f in self._threads_to_spawn:
            self._threads.append(gevent.spawn(f))
        self._threads.append(gevent.spawn(self.run))

    def stop(self):
        """ Stops the module. """
        self.is_running = False
        for g in self._threads:
            g.kill()
        self._threads = []

    def get_request_handle(self):
        """
        Returns an integer to be used as the id of a request. Requests sent
        from a module must have unique id's so that their responses can be
        properly handled, so this function is used to generate those id's.
        """
        self._next_request_handle += 1
        return self._next_request_handle

    def ask_async(self, dest_mod_id):
        """
        Returns a `AsyncRequester` that can be used to send requests to the
        module with id `dest_mod_id`.
        """
        dest_mod = Module.get_by_id(dest_mod_id)
        async_requester = AsyncRequester(self, dest_mod)
        return async_requester

    def ask(self, dest_mod_id):
        """
        Returns a `Requester` that can be used to send requests to the module
        with id `dest_mod_id`.
        """
        dest_mod = Module.get_by_id(dest_mod_id)
        requester = Requester(self, dest_mod)
        return requester

    def on_request(self, item):
        """
        Callback for requests. Processes the request and then sends a
        response back to the module that sent the request.
        """
        fn_name, fn_args, fn_kwargs = item.value
        try:
            value = self.process_request(fn_name, fn_args, fn_kwargs)
        except Exception as e:
            value = e
        src_mod = Module.get_by_id(item.src_id)
        result = StreamItem(
            value, InternalVariable.RESPONSE, time.time(), self.mod_id,
            item.object_id
        )
        src_mod._responses.put(result)

    def on_response(self, item):
        """
        Callback for responses. Sets the value of the AsyncResult returned
        from the original request.
        """
        result = self._open_requests.pop(item.object_id)
        if isinstance(item.value, Exception):
            result.set_exception(item.value)
        else:
            result.set(item.value)

    def process_request(self, fn_name, fn_args, fn_kwargs):
        """
        If there is an endpoint on this module with the name `fn_name`,
        then that function is called with the arguments `fn_args`. Otherwise,
        an exception is returned.
        """
        if fn_name in self._info.endpoints:
            endpoint = getattr(self, fn_name)
            return endpoint(*fn_args, **fn_kwargs)
        else:
            return RuntimeError()

    def run(self):
        """
        This function runs the `Module`. Subclasses of `Module` can override
        this function to define the normal operation of the module. There is a
        default implementation that simply spawns reader threads for each of
        the inputs and waits for them to terminate.
        """
        pass

    def debug(self, msg, *args, **kwargs):
        """
        Logs a message with level logging.DEBUG on the logger for this module
        """
        return self._logger.debug(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        """
        Logs a message with level logging.INFO on the logger for this module
        """
        return self._logger.info(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        """
        Logs a message with level logging.WARNING on the logger for this module
        """
        return self._logger.warning(msg, *args, **kwargs)

    def error(self, msg, *args, **kwargs):
        """
        Logs a message with level logging.ERROR on the logger for this module
        """
        return self._logger.error(msg, *args, **kwargs)

    def critical(self, msg, *args, **kwargs):
        """
        Logs a message with level logging.CRITICAL on the logger for this
        module
        """
        return self._logger.critical(msg, *args, **kwargs)

Module._registry = {}
