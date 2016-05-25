function (key, values, rereduce) {
  var best = values[0];
  var num_vals = values.length;
  for (var i = 0; i < num_vals; i++) {
    var curr_val = values[i];
    if (curr_val.timestamp > best.timestamp) {
      best = curr_val;
    }
  }
  return best;
}

