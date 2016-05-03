function (key, values, rereduce) {
  var best = null;
  var num_vals = values.length;
  for (var i = 0; i < num_vals; i++) {
    var curr_val = values[i];
    if (best == null || curr_val.timesamp < best.timestamp) {
      best = curr_val;
    }
  }
  return best;
}

