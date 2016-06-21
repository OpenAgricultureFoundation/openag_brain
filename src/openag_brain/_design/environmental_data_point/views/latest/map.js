function (doc) {
  var point_type;
  if (doc.is_desired) {
    point_type = 'desired';
  }
  else {
    point_type = 'measured';
  }
  emit([doc.environment, doc.variable, point_type], doc);
}
