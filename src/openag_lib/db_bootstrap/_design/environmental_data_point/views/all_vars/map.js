function (doc) {
  var all_var_types = [
    "air_temperature",
    "air_carbon_dioxide",
    "air_humidity",
    "air_oxygen",
    "water_temperature",
    "water_potential_hydrogen",
    "water_electrical_conductivity"
  ];
  if(all_var_types.indexOf(doc.variable) == -1){
    return;
  }
  var point_type;
  if (doc.is_desired) {
    point_type = 'desired';
  }
  else {
    point_type = 'measured';
  }
  emit([doc.environment, point_type, doc.variable, doc.timestamp], doc);
}
