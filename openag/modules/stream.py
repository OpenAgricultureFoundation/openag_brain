from collections import namedtuple

__all__ = ['GeneralStreamItem', 'TypedStreamItem', 'SourcedStreamItem']

GeneralStreamItem = namedtuple(
    'GeneralStreamItem', ['src_id', 'data_type', 'value']
)
TypedStreamItem = namedtuple('TypesStreamItem', ['data_type', 'value'])
SourcedStreamItem = namedtuple('SourcedStreamItem', ['src_id', 'value'])
