#version 120

// Passes the vertex pick color 

uniform vec4 pick_color;

void main()
{
  gl_FragColor = pick_color;
}
