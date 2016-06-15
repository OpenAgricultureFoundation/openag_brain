function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  var required_fields = ['environment', 'type'];
  var field;
  for (var i in required_fields) {
    field = required_fields[i];
    if (!newDoc.hasOwnProperty(field)) {
      throw({forbidden: "FirmwareModule instances are required to have a " + field + " field"});
    }
  }
}
