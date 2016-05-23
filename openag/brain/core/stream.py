"""
This module defines a single class `StreamItem`, which is the only type of
object that is allowed to be send between modules.
"""
import json

__all__ = ['StreamItem']

class StreamItem:
    """
    Instances of this class have 5 attribute: `value`, `data_type`,
    `timestamp`, `src_id`, and (optionally) `object_id`. `value is the value
    being sent. `data_type` is a DataType object identifying the type of data
    being sent. `timestamp` is the timestamp associated with the value.
    `src_id` is the id of the module that sent the item. `object_id` is the id
    of the object for which this value was recorded.
    """

    def __init__(self, value, data_type, timestamp, src_id, object_id=None):
        self.value = value
        self.data_type = data_type
        self.timestamp = timestamp
        self.src_id = src_id
        self.object_id = object_id

    def __str__(self):
        return json.dumps({
            'value': str(self.value),
            'data_type': self.data_type,
            'timestamp': self.timestamp,
            'src_id': self.src_id,
            'object_id': self.object_id
        })
