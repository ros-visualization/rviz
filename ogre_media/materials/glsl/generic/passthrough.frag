#version 120

// Passes the vertex color on unchanged.

//in vec4 gl_Color;
//out vec4 gl_FragColor;

void main()
{
  gl_FragColor = gl_Color;
}
