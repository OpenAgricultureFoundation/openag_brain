"""
Constants for openag_brain.
"""

"""
A null setpoint sentinal value (magic number).
We use it to signal to the controller that the feedback loop should
be broken.
"""
NULL_SETPOINT_SENTINAL = -9000000009.00001

"""
The set of all sentinals. Used to check if a number is any sentinal.
"""
SENTINALS = frozenset((
    NULL_SETPOINT_SENTINAL
))
