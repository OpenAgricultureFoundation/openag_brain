function(doc) {
  num_groups = doc.groups.length;
  for (var i = 0; i < num_groups; i++) {
    emit(doc.groups[i], doc._id);
  }
}
