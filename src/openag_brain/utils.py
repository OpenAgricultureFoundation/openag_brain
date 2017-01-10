import os
import rospkg
from sys import maxsize
from random import randint

def resolve_fixtures(fixtures):
    """
    Given a list of fixture names, returns a list of fully resolved fixture
    paths.
    """
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('openag_brain')
    fixtures_path = os.path.join(pkg_path, "fixtures")
    resolved = [os.path.join(fixtures_path, fixture_name + ".json") for fixture_name in fixtures]
    return resolved

def gen_doc_id(curr_time):
    """
    Given a unix time, generate a unique ID string with extremely low chance
    of collision.

    Returns a string.
    """
    return "{}-{}".format(curr_time, randint(0, maxsize))