#version 120

// Splits the fragment's floating point R value into the output
// RGB value, so it can be interpreted as fixed-point value.

//includes:
vec4 packDepth_impl( float normalized_depth );

void main()
{
  gl_FragColor = packDepth_impl( gl_Color.r );
}
