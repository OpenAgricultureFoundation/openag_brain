function (head, req) {
  var headers = ["value", "timestamp"];
  start({'headers': {'Content-Type': 'text/csv; charset=utf-8; header=present'}});
  send('"' + headers.join('","') + '"\n');
  while (r=getRow()) {
    headers.forEach(function(v,i) {
      send(r.value[v]);
      (i + 1 < headers.length) ? send(',') : send('\n');
    })
  }
}
