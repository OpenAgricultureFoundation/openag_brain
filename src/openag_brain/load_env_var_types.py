import rospy


class VariableInfo(dict):
    items = {}

    def __init__(self, name, description, units=None):
        self.name = name
        self.__doc__ = description
        self.units = units

    def __str__(self):
        return self.name

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
    for item in var_dict.items.keys():
        variable = VariableInfo(item['name'], var['description'], var['units'])
        variables.append(variable)
    return tuple(variables)
