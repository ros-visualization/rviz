#version 120

// Generic vertex shader for point sprites
// sets position and point size.
// Works for perspective and orthographic projection.

uniform mat4 worldviewproj_matrix;
uniform vec4 size;

#ifdef WITH_DEPTH
  //include:
  void passDepth( vec4 pos );
#endif

void main()
{
  gl_Position = worldviewproj_matrix * gl_Vertex;
  gl_FrontColor = gl_Color;
  gl_PointSize = size.x;

#ifdef WITH_DEPTH
  passDepth( gl_Vertex );
#endif
}
