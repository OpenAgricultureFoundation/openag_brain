
MOCK_RECIPE_SIMPLE_A = {
            "_id": "air_water_test",
            "format": "simple",
            "version": "0.0.1",
            "operations": [
                            [0, "air_temperature", 24],
                            [0, "water_temperature", 22],
                            [30, "air_temperature", 24],  # Lesson Learned interpret_simple_recipe calculates the end time by the last time in the list
                            [30, "water_temperature", 22]
                          ]
            }

MOCK_RECIPE_SIMPLE_B = {
  "_id": "test",
  "version": "6.1.7",
  "format": "simple",
  "operations": [
    # recipe_start
    [ 0, "one", 1 ],
    [ 0, "two", 2 ],
    [ 0, "three", 3 ],
    [ 60, "four", 4 ],
    [ 60, "five", 5 ],
    [ 120, "six", 6 ],
    [ 120, "seven", 7 ],
    [ 180, "eight", 8 ],
    # recipe_end
  ]
}

MOCK_RECIPE_FLEXFORMAT_A = {
    "_id": "general_greens",
    "format": "flexformat",
    "version": "1.0",
    "seeds": ["green_lettuce_seed", "romaine_lettuce_seed"],
    "plant_type": ["lettuce", "green"],
    "certified_by": ["SuperSmartScientist"],
    "optimization": ["general purpose"],
    "author": "John Doe",
    "rating": 20,
    "downloads": 10000,
    "date_created": "2017-02-08",
    "phases": [
          { "name": "early",
            "cycles": 14,
            "time_units" : "hours",
            "variable_units": {"air_temperature": "Celcius",
                               "nutrient_flora_duo_a": "ml",
                               "nutrient_flora_duo_b": "ml",
                               "light_illuminance": "percent_relative"},
            "step": { "air_temperature": [{"start_time": 0, "end_time": 6, "value": 20},
                                          {"start_time": 6, "end_time": 18, "value": 23},
                                          {"start_time": 18, "end_time": 24, "value": 19}],
                      "nutrient_flora_duo_a": [{"start_time": 0, "end_time": 6, "value": 5},
                                               {"start_time": 6, "end_time": 18, "value": 2},
                                               {"start_time": 18, "end_time": 24, "value": 5}],
                      "nutrient_flora_duo_b": [{"start_time": 0, "end_time": 6, "value": 2}],
                      "light_illuminance": [{"start_time": 0, "end_time": 6, "value": 4},
                                            {"start_time": 18, "end_time": 24, "value": 3}],
                },
          },
          { "name": "middle",
            "cycles": 20,
            "time_units" : "hours",
            "variable_units": {"air_temperature": "Celcius",
                               "nutrient_flora_duo_a": "ml",
                               "nutrient_flora_duo_b": "ml",
                               "light_illuminance": "percent_relative"},
            "step": { "air_temperature": [{"start_time": 0, "end_time": 6, "value": 20},
                                          {"start_time": 6, "end_time": 18, "value": 23},
                                          {"start_time": 18, "end_time": 24, "value": 19}],
                      "nutrient_flora_duo_a": [{"start_time": 0, "end_time": 6, "value": 5},
                                               {"start_time": 6, "end_time": 18, "value": 2},
                                               {"start_time": 18, "end_time": 24, "value": 5}],
                      "nutrient_flora_duo_b": [{"start_time": 0, "end_time": 6, "value": 2}],
                      "light_illuminance": [{"start_time": 0, "end_time": 6, "value": 4},
                                            {"start_time": 18, "end_time": 24, "value": 3}]
                },
          },
          { "name": "late",
            "cycles": 7,
            "time_units" : "hours",
            "variable_units": {"air_temperature": "Celcius",
                               "nutrient_flora_duo_a": "ml",
                               "nutrient_flora_duo_b": "ml",
                               "light_illuminance": "percent_relative"},
            "step": {
                      "air_temperature": [{"start_time": 0, "end_time": 6, "value": 20},
                                          {"start_time": 6, "end_time": 18, "value": 23},
                                          {"start_time": 18, "end_time": 24, "value": 19}],
                      "nutrient_flora_duo_a": [{"start_time": 0, "end_time": 6, "value": 5},
                                               {"start_time": 6, "end_time": 18, "value": 2},
                                               {"start_time": 18, "end_time": 24, "value": 5}],
                      "nutrient_flora_duo_b": [{"start_time": 0, "end_time": 6, "value": 2}],
                      "light_illuminance": [{"start_time": 0, "end_time": 6, "value": 4},
                                            {"start_time": 18, "end_time": 24, "value": 3}]
                },
          }
        ]
  }
