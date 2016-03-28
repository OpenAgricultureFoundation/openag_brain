"""
This module defines a few data structures used for passing messages between
modules. In particular, `OutputChannel`s always write `GeneralStreamItem`s to
`InputQueue`s, but `InputQueue`s can provide their `Module`s with a different
view of the stream in the form of `TypedStreamItem`s, `SourcesStreamItem`s or
raw values.
"""
from collections import namedtuple

__all__ = ['GeneralStreamItem', 'TypedStreamItem', 'SourcedStreamItem']

GeneralStreamItem = namedtuple(
    'GeneralStreamItem', ['src_id', 'data_type', 'value']
)
TypedStreamItem = namedtuple('TypesStreamItem', ['data_type', 'value'])
SourcedStreamItem = namedtuple('SourcedStreamItem', ['src_id', 'value'])
