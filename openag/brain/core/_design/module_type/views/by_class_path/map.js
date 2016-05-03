function (doc) {
  if (doc.package_path && doc.class_name) {
    emit(doc.package_path + ":" + doc.class_name, doc._id);
  }
}
