"""
This module stores some ROS parameter names used for passing information
between modules in the system.
"""

CURRENT_RECIPE = "current_recipe"
"""
Stores the ID of the currently running recipe or "" if no recipe is running
"""
CURRENT_RECIPE_START = "current_recipe_start"
"""
Stores the UNIX timestamp at which the current recipe was started or 0 if no
recipe is running
"""
SUPPORTED_RECIPE_FORMATS = "supported_recipe_formats"
"""
Stores a comma-separated list of all of the recipe formats supported by the
running recipe handler module
"""
CATEGORIES = "categories"
"""
Stores a list of the currently active module categories
"""
