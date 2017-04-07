from src.openag_brain.load_env_var_types import create_variables


var_dict = {'air_temp': {'units': 'C', 'name': 'air_temp', 'description': 'asdjalksjdf;asjkd'},
             'air_press': {'units': 'mmhg', 'name': 'air_pres', 'description': 'asdfasdfawerwernvcx'}}

def test_create_variables():
    variables = create_variables(var_dict)
    var_names = [str(var) for var in variables]
    print(variables[0].name)
    assert var_names == ['air_temp', 'air_press']
    print(var_dict[variables[0]])
