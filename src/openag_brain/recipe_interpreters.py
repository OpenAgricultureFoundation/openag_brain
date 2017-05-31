import rospy
from openag_brain.utils import trace

RECIPE_START = VariableInfo.from_dict(
    rospy.get_param('/var_types/recipe_variables/recipe_start'))

RECIPE_END = VariableInfo.from_dict(
    rospy.get_param('/var_types/recipe_variables/recipe_end'))

def interpret_simple_recipe(recipe, start_time, now_time):
    """
    Produces a tuple of ``(variable, value)`` pairs by building up
    a recipe state from walking through the recipe keyframes
    """
    _id = recipe["_id"]
    operations = recipe["operations"]
    rospy.loginfo(operations)
    end_time_relative = operations[-1][0]
    trace("recipe_handler: interpret_simple_recipe end_time_relative=%s",
        end_time_relative)
    end_time = start_time + end_time_relative
    # If start time is at some point in the future beyond the threshold
    if start_time - now_time > THRESHOLD:
        raise ValueError("Recipes cannot be scheduled for the future")
    # If there are no recipe operations, immediately start and stop
    # The recipe.
    if not len(operations):
        return (
            (RECIPE_START.name, _id),
            (RECIPE_END.name, _id),
        )
    if now_time >= (end_time + THRESHOLD):
        return ((RECIPE_END.name, _id),)
    if abs(now_time - start_time) < THRESHOLD:
        return ((RECIPE_START.name, _id),)

    now_relative = (now_time - start_time)

    # Create a state object to accrue recipe setpoint values.
    state = {}
    # Build up state up until now_time (inclusive).
    trace("recipe_handler: interpret_simple_recipe now=%s", now_relative)
    for timestamp, variable, value in operations:
        if timestamp > now_relative:
            break
        state[variable] = value
        trace("recipe_handler: interpret_simple_recipe: %s %s %s",
            timestamp, variable, value)
    rospy.loginfo(state)
    return tuple(
        (variable, value)
        for variable, value in state.iteritems()
    )
