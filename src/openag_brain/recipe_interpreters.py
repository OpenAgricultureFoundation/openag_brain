import rospy
from openag_brain.load_env_var_types import VariableInfo
from openag_brain.utils import trace
from datetime import datetime

RECIPE_START = VariableInfo.from_dict(
    rospy.get_param('/var_types/recipe_variables/recipe_start'))

RECIPE_END = VariableInfo.from_dict(
    rospy.get_param('/var_types/recipe_variables/recipe_end'))

# A threshold to compare time values in seconds.
THRESHOLD = 1

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


def interpret_flexformat_recipe(recipe, current_time, start_time):
    """
    Recipe Interpreter should read a recipe, current_time, start_time, variable and return a value.
       Determine the time since the beginning of the recipe.
       Determine what is the current step
       Calculate the remaining time left in this step.
       Look up the value within that step for that variable.
    """

    # Returns a list of the phases and step durations  [(duration_of_phase_1, duration_of_step_1), (duration_of_phase_2, duration_of_step_2), etc]
    duration_of_phases_steps = calc_duration_of_phases_steps(recipe['phases'])
    current_phase_number, duration_in_step = calc_phase_and_time_remaining(duration_of_phases_steps,
                                                                           current_time,
                                                                           start_time)
    current_phase = recipe['phases'][current_phase_number]
    state = {}
    for variable, variable_step_data in current_phase['step'].items():
        value = determine_value_for_step(variable_step_data, duration_in_step)
        state[variable] = value
    return tuple(
        (variable, value)
        for variable, value in state.iteritems()
    )



def determine_value_for_step(variable_step_data, duration_in_step):
    """
    variable_step_data = [{"start_time": 0, "end_time": 6, "value": 20},
                          {"start_time": 6, "end_time": 18, "value": 23},
                          {"start_time": 18, "end_time": 24, "value": 19}]
    duration_in_step = 15
    Given the time within a step, what value is expected.
    """

    for row in variable_step_data:
        if row['start_time'] <= duration_in_step <= row['end_time']:
            return row['value']


def calculate_max_duration_from_step(step):
    """
    Determines the total duration of this step. Normally it is 24 hours.
       Could add other validation steps here as well.
       Convert to numpy and use argmax
    """
    max_time = 0
    for variable, start_end_times in step.items():
        for data in start_end_times:
            if data['start_time'] > data['end_time']:
                raise Exception("Start_time is after end time.")
            elif max_time < max(data['start_time'], data['end_time']):
                max_time = max(data['start_time'], data['end_time'])
    return max_time


def calc_duration_of_phases_steps(phases):
    """
    Returns a list with the duration of the step and the entire phase
    """
    duration_of_phases_steps = []
    for phase in phases:
        cycles = phase['cycles']
        max_duration = calculate_max_duration_from_step(phase['step'])
        duration_of_phases_steps.append((cycles*max_duration, max_duration))
    return duration_of_phases_steps


def calc_phase_and_time_remaining(duration_of_phases_steps, current_time, start_time):
    """
    duration_of_phases_steps == [(336, 24), (480, 24), (168, 24)]
    current_time : datetime : UTC of the current time
    start_time : datetime : UTC time the recipe started
    return: current_phase_number, duration_in_phase
    """
    time_since_start = current_time - start_time
    time_since_start = time_since_start.total_seconds() / 3600

    time_remaining = time_since_start
    for i, (total_duration, step_duration) in enumerate(duration_of_phases_steps):
        if time_remaining > total_duration:
            time_remaining -= total_duration
        else:
            duration_in_phase = time_remaining % step_duration
            current_phase_number = i
            break
    return current_phase_number, duration_in_phase
