from src.openag_brain.load_env_var_types import create_variables, VariableInfo


var_dict = {'air_temp': {'units': 'C', 'name': 'air_temp', 'description': 'asdjalksjdf;asjkd'},
             'air_press': {'units': 'mmhg', 'name': 'air_pres', 'description': 'asdfasdfawerwernvcx'}}

def test_create_variables():
    variables = create_variables(var_dict)
    print(variables["air_temp"].name)
    assert isinstance(variables["air_temp"], VariableInfo)
    assert isinstance(variables["air_press"], VariableInfo)
    assert variables["air_temp"].name == "air_temp"
    assert variables["air_temp"].description == var_dict["air_temp"]["description"]
    assert variables["air_temp"].units == var_dict["air_temp"]["units"]