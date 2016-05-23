function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  if (!newDoc.input_module) {
    throw({forbidden: "Module instances are required to have a 'input_module' attribute"})
  }
  if (!newDoc.input_name) {
    throw({forbidden: "Module instances are required to have a 'input_name' attribute"})
  }
  if (!newDoc.output_module) {
    throw({forbidden: "Module instances are required to have a 'output_module' attribute"})
  }
  if (!newDoc.output_name) {
    throw({forbidden: "Module instances are required to have a 'output_name' attribute"})
  }
}
