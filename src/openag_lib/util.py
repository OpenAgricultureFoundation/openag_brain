"""
Shared utilities for openag scripts.
"""
import os
import re
from urlparse import urlparse

def index_by_id(docs):
    """Index a list of docs using `_id` field.
    Returns a dictionary keyed by _id."""
    return {doc["_id"]: doc for doc in docs}

def dedupe_by(things, key=None):
    """
    Given an iterator of things and an optional key generation function, return
    a new iterator of deduped things. Things are compared and de-duped by the
    key function, which is hash() by default.
    """
    if not key:
        key = hash
    index = {key(thing): thing for thing in things}
    return index.values()

def parent_dirname(file_path):
    return os.path.basename(os.path.dirname(file_path))

def make_dir_name_from_url(url):
    """This function attempts to emulate something like Git's "humanish"
    directory naming for clone. It's probably not a perfect facimile,
    but it's close."""
    url_path = urlparse(url).path
    head, tail = os.path.split(url_path)
    # If tail happens to be empty as in case `/foo/`, use foo.
    # If we are looking at a valid but ugly path such as
    # `/foo/.git`, use the "foo" not the ".git".
    if len(tail) is 0 or tail[0] is ".":
        head, tail = os.path.split(head)
    dir_name, ext = os.path.splitext(tail)
    return dir_name

# C++ keywords list
CPP_KEYWORDS = [
    "alignas", "alignof", "and", "and_eq", "asm", "atomic_cancel",
    "atomic_commit", "atomic_noexcept", "auto", "bitand", "bitor", "bool",
    "break", "case", "catch", "char", "char16_t", "char32_t", "class",
    "compl", "concept", "const", "constexpr", "const_cast", "continue",
    "decltype", "default", "delete", "do", "double", "dynamic_cast",
    "else", "enum", "explicit", "export", "extern", "false", "float",
    "for", "friend", "goto", "if", "inline", "int", "long", "mutable",
    "namespace", "new", "noexcept", "not", "not_eq", "nullptr", "operator",
    "or", "or_eq", "private", "protected", "public", "register",
    "reinterpret_cast", "requires", "return short", "signed", "sizeof",
    "static", "static_assert", "static_cast", "struct", "switch",
    "synchronized", "template", "this", "thread_local", "throw", "true",
    "try", "typedef", "typeid", "typename", "union", "unsigned", "using",
    "virtual", "void", "volatile", "wchar_t", "while", "xor", "xor_eq"
]

def safe_cpp_var(s):
    """
    Given a string representing a variable, return a new string that is safe
    for C++ codegen. If string is already safe, will leave it alone.
    """
    s = str(s)
    # Remove non-word, non-space characters
    s = re.sub(r"[^\w\s]", '', s)
    # Replace spaces with _
    s = re.sub(r"\s+", '_', s)
    # Prefix with underscore if what is left is a reserved word
    s = "_" + s if s in CPP_KEYWORDS or s[0].isdigit() else s
    return s