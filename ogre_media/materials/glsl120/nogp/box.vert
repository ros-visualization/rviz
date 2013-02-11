#version 120

// Computes the position of a box vertex from its texture coords.
// Used in case that geometry shaders are not supported.

uniform mat4 worldviewproj_matrix;
uniform vec4 camera_pos;
uniform vec4 size;
uniform vec4 auto_size;

#ifdef WITH_DEPTH
  //include:
  void passDepth( vec4 pos );
#endif

void main()
{
  // if auto_size == 1, then size_factor == size*gl_Vertex.z
  // if auto_size == 0, then size_factor == size
  vec4 size_factor = (1-auto_size.x+(auto_size.x*gl_Vertex.z))*size;

  vec4 s = gl_MultiTexCoord0 * size_factor;
  vec4 pos = gl_Vertex - s;
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = gl_MultiTexCoord0;
  gl_FrontColor = gl_Color;

#ifdef WITH_DEPTH
  passDepth( pos );
#endif
}
