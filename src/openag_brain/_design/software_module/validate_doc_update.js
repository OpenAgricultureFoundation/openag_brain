function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  if (!newDoc.type) {
    throw({forbidden: "SoftwareModule instances are required to have a 'type' attribute"})
  }
}
