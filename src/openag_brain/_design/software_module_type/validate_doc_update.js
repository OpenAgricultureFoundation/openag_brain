function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  var required_fields = ['package', 'executable', 'description', 'arguments'];
  var field;
  for (var i in required_fields) {
    field = required_fields[i];
    if (!newDoc.hasOwnProperty(field)) {
      throw({forbidden: "SoftwareModuleType instances are required to have a " + field + " field"});
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
}
