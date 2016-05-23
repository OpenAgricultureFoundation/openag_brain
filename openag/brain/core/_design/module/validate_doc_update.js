function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  if (!newDoc.type) {
    throw({forbidden: "Module instances are required to have a 'type' attribute"})
  }
}
