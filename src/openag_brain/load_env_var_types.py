class VariableInfo(dict):
    @staticmethod
    def from_dict(desc_dict):
        """
        Construct a variable from the description dictionaries we have
        in the yaml file.
        """
        return VariableInfo(
            desc_dict["name"],
            desc_dict["description"],
            type=desc_dict.get("type", None),
            units=desc_dict.get("units", None)
        )

    def __init__(self, name, description, type=None, units=None):
        self.name = name
        self.__doc__ = description
        self.units = units
        # A valid ROS type string, like std_msgs/Float64
        self.type = type
    def __key(self):
        return self.name
    def __str__(self):
        return self.name
    def __repr__(self):
        return self.name
    def __hash__(self):     # Needed to be used as a key in a dictionary lookup
        return hash(self.name)
    def __eq__(x, y):       # Needed to be used as a key in a dictionary lookup
        return x.name == y

# env_var = rospy.get_param('/environment_variables')
# recipe_var = rospy.get_param('/recipe_variables')
# camera_var = rospy.get_param('/camera_variables')
# user_var = rospy.get_param('/user_variables')

def create_variables(var_dict):
    """
    Converts each dictionary item into the VariableInfo class to allow for them
    to be referenced by name directly without having to look up the name key.
    For example, print(variable) instead of print(variable['name']).
    Returns a dict of VariableInfo instances.

    :param: var_dict : dictionary with each value being a dictionary containing
        name, description, units
    """
    ## Need to change this later so it is generic to work for any number of
    ## items in the dictionary, so this file doesn't need to be modified when
    ## new items are added to the var_types definition.
    return {
        name: VariableInfo.from_dict(desc_dict)
        for name, desc_dict in var_dict.iteritems()
    }
