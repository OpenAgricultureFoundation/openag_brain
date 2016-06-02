function(newDoc, oldDoc, userCtx, secObj) {
  if (newDoc._deleted) {
    return;
  }
  if (!newDoc.environment) {
    throw({forbidden: "EnvironmentalDataPoint instances are required to have an 'environment'attribute"});
  }
  if (!newDoc.variable) {
    throw({forbidden: "EnvironmentalDataPoint instances are required to have a 'variable'attribute"});
  }
  if (newDoc.is_desired === null) {
    throw({forbidden: "EnvironmentalDataPoint instances are required to have an 'is_desired'attribute"});
  }
  if (!newDoc.value) {
    throw({forbidden: "EnvironmentalDataPoint instances are required to have a 'value'attribute"});
  }
  if (!newDoc.timestamp) {
    throw({forbidden: "EnvironmentalDataPoint instances are required to have a 'timestamp'attribute"});
  }
}
