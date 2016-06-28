function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  var required_fields = ['pio_id', 'header_file', 'class_name', 'description', 'arguments', 'inputs', 'outputs'];
  var field;
  for (var i in required_fields) {
    field = required_fields[i];
    if (!newDoc.hasOwnProperty(field)) {
      throw({forbidden: "FirmwareModuleType instances are required to have a " + field + " field"});
    }
  }
  var required_argument_fields = ["name", "type"];
  for (var i in required_argument_fields) {
    field = required_argument_fields[i];
    for (var param in newDoc.arguments) {
      if (!newDoc.arguments[param].hasOwnProperty(field)) {
        throw({forbidden: "argument " + param + " is missing a " + field + " field"});
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
}
