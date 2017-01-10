"""
Clojure-esque multimethods.

@NOTE if we ever upgrade to Python 3, we should consider using
functools.singledispatch instead. It should do the trick.
"""
from functools import wraps

def multidispatch(key):
    """
    Clojure-style fancy multimethods as a Python decorator.
    Clojure-style Multimethods are able to dispatch on any combination of
    arguments by producing a key via a key function. If no key is found for
    the argument combo, a default function is called.

    Note that key function has to consume all positional arguments, but
    ignores keyword arguments.

    Usage::

        @multidispatch(lambda x: type(x).__name__)
        def add(x): return 'default'

        @add.register('float')
        def add_float(x): return 'float'

        add(1.0) # 'float'

    All registered methods are stored in a dict, assigned to the `methods`
    property of the returned function, so you can inspect them::

        add.methods.keys() # ['float']
    """
    def decorate_default(f):
        methods = {}
        @wraps(f)
        def multi(*args, **kwargs):
            method = methods.get(key(*args), f)
            return method(*args, **kwargs)
        def register(k):
            """
            Register a function at key by using this function as a decorator
            factory
            """
            def decorate(f):
                methods[k] = f
                return f
            return decorate
        multi.register = register
        multi.methods = methods
        return multi
    return decorate_default
