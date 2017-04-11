class VariableInfo(dict):
    def __init__(self, name, description, _type=None, units=None):
        self.name = name
        self.__doc__ = description
        self.units = units
        self.type = _type
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
    For example, print(variable) instead of print(variable['name'])
    :param: var_dict : dictionary with each value being a dictionary containing
        name, description, units
    """
    variables = []
    ## Need to change this later so it is generic to work for any number of
    ## items in the dictionary, so this file doesn't need to be modified when
    ## new items are added to the var_types definition.
    for name, value in var_dict.items():
        if 'units' in value:
            if 'type' in value:
                var = VariableInfo(name, value['description'], value['type'],
                               value['units'])
            else:
                var = VariableInfo(name, value['description'], value['units'])
        else:
            var = VariableInfo(name, value['description'])
        variables.append(var)
    return variables
