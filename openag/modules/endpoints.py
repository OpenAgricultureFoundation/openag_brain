__all__ = ['endpoint', 'Procedure']

def endpoint(f):
    """ Marks the function `f` as an endpoint of its module """
    f.is_endpoint = True
    return f

class Procedure:
    """
    A list of endpoints that should be called in order by a client. Contains
    a `desciption` and a list of the `endpoints` to call.
    """
    def __init__(self, endpoints, description):
        self.endpoints = [endpoint.__name__ for endpoint in endpoints]
        self.description = description

    def to_dict(self):
        """ Returns a dictionary describing this object """
        return {
            'endpoints': self.endpoints,
            'description': self.description
        }
