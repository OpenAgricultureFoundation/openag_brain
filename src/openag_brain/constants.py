"""
Constants for openag_brain.
"""

"""
A null setpoint sentinel value (magic number).
We use it to signal to the controller that the feedback loop should
be broken.

https://en.wikipedia.org/wiki/Sentinel_value
"""
NULL_SETPOINT_SENTINEL = -9000000009.00001

"""
The set of all sentinel. Used to check if a number is any kind of sentinel.
"""
SENTINELS = frozenset((
    # Don't forget the trailing comma (making this a tuple not a grouping)
    NULL_SETPOINT_SENTINEL,
))
