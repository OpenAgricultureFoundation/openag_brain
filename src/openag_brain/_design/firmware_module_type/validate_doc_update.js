function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  var required_fields = ['url', 'header_file', 'class_name', 'description', 'parameters', 'inputs', 'outputs'];
  var field;
  for (var i in required_fields) {
    field = required_fields[i];
    if (!newDoc.hasOwnProperty(field)) {
      throw({forbidden: "FirmwareModuleType instances are required to have a " + field + " field"});
    }
  }
}
