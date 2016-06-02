function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  if (!newDoc.description) {
    throw({forbidden: "SoftwareModuleType instances are required to have a 'description' attribute"})
  }
  if (!newDoc.parameters) {
    throw({forbidden: "SoftwareModuleType instances are required to have a 'parameters' attribute"})
  }
  if (!newDoc.inputs) {
    throw({forbidden: "SoftwareModuleType instances are required to have an 'inputs' attribute"})
  }
  if (!newDoc.outputs) {
    throw({forbidden: "SoftwareModuleType instances are required to have an 'outputs' attribute"})
  }
  if (!newDoc.services) {
    throw({forbidden: "SoftwareModuleType instances are required to have a 'services' attribute"})
  }
}
