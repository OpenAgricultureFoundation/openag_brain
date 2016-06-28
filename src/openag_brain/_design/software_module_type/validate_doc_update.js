function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  var required_fields = ['package', 'executable', 'description', 'parameters', 'inputs', 'outputs', 'services'];
  var field;
  for (var i in required_fields) {
    field = required_fields[i];
    if (!newDoc.hasOwnProperty(field)) {
      throw({forbidden: "SoftwareModuleType instances are required to have a " + field + " field"});
    }
  }
  var required_parameter_fields = ["type"];
  for (var i in required_parameter_fields) {
    field = required_parameter_fields[i];
    for (var param in newDoc.parameters) {
      if (!newDoc.parameters[param].hasOwnProperty(field)) {
        throw({forbidden: "Parameter " + param + " is missing a " + field + " field"});
      }
    }
  }
  var required_input_fields = ["type"];
  for (var i in required_input_fields) {
    field = required_input_fields[i];
    for (var input in newDoc.inputs) {
      if (!newDoc.inputs[input].hasOwnProperty(field)) {
        throw({forbidden: "input " + input + " is missing a " + field + " field"});
      }
    }
  }
  var required_output_fields = ["type"];
  for (var i in required_output_fields) {
    field = required_output_fields[i];
    for (var output in newDoc.outputs) {
      if (!newDoc.outputs[output].hasOwnProperty(field)) {
        throw({forbidden: "output " + output + " is missing a " + field + " field"});
      }
    }
  }
  var required_service_fields = ["type"];
  for (var i in required_service_fields) {
    field = required_service_fields[i];
    for (var service in newDoc.services) {
      if (!newDoc.services[service].hasOwnProperty(field)) {
        throw({forbidden: "service " + service + " is missing a " + field + " field"});
      }
    }
  }
}
