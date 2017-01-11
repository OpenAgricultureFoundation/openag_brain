"""
Implements naive unbounded memoization.

@NOTE if we ever switch to Python 3.x, we should use functools.lru_cache
decorator instead which serves the same purpose (and with more nifty features).
"""
from functools import wraps

def memoize(f):
    """
    Memoize a function based on the arguments tuple passed in. Example::

        @memoize
        def foo(x, y): return x + y
        foo(1, 2) # 3

    Memoized function uses identiy of arguments to cache results, so arguments
    must be hashable, and only positional arguments are supported.

    Memoized function's cache is stored on the `cache` property of the
    function, so you can also inspect its memoized values and clear the cache.

        foo.cache.items() # [((1, 2), 3)]
        foo.cache.clear()
        foo.cache.items() # []
    """
    cache = {}
    @wraps(f)
    def memo(*args):
        if not args in cache:
            cache[args] = f(*args)
        return cache[args]
    # Expose cache on function so it can be inspected and cleared.
    memo.cache = cache
    return memo
