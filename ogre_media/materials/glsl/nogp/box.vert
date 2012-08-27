#version 120

// Computes the position of a box vertex from its texture coords.
// Used in case that geometry shaders are not supported.

uniform mat4 worldviewproj_matrix;
uniform vec4 camera_pos;
uniform vec4 size;

#ifdef WITH_DEPTH
  //include:
  void passDepth( vec4 pos );
#endif

void main()
{
  vec4 s = gl_MultiTexCoord0 * size;
  vec4 pos = gl_Vertex - s;
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = gl_MultiTexCoord0;
  gl_FrontColor = gl_Color;

#ifdef WITH_DEPTH
  passDepth( pos );
#endif
}
