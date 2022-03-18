
function rotateY(t) {
  var s = Math.sin(t);
  var c = Math.cos(t);
  return [
    c, 0,-s, 0,
    0, 1, 0, 0,
    s, 0, c, 0,
    0, 0, 0, 1
  ];
}

function perspective(fovy, aspect, near, far) {
  var y = 1 / Math.tan(fovy / 2 * Math.PI / 180);
  var x = y / aspect;
  var c = (near + far) / (near - far);
  var d = (near * far) / (near - far) * 2;
  return [
    x, 0, 0, 0,
    0, y, 0, 0,
    0, 0, c,-1,
    0, 0, d, 0
  ];
}
