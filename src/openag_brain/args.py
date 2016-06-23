"""
This module contains some standard command line arguments to be used by scripts
in this project. In particular, it defines a set of functions that take as
input an `argparse.ArgumentParser` instance and register a specific argument on
the instance. Collecting these in one location makes it easy to update e.g. the
description of an argument in one place instead of in every script that uses
that argument.
"""
__all__ = [
    'add_db_server_arg', 'add_hostname_arg'
]

def add_db_server_arg(parser):
    parser.add_argument(
        "-D", "--db_server", help="Address of the database server",
        default="http://localhost:5984"
    )

def add_hostname_arg(parser):
    parser.add_argument(
        "-H", "--hostname", help="Hostname of the machine running this file",
        default="localhost"
    )
