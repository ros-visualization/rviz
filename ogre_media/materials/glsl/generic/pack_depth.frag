#version 120

// Splits the fragment's floating point R value into the output
// RGB value, so it can be interpreted as fixed-point value.

//includes:
vec4 packDepth_impl( float z );

//in vec4 gl_Color;
//out vec4 gl_FragColor;

void main()
{
  gl_FragColor = packDepth_impl( gl_Color.r );
}
