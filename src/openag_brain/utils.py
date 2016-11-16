import rospkg

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