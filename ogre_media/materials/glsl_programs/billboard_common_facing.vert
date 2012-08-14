#version 120

// Computes the position of a billboard vertex so
// the resulting quad will face a given direction given by up & normal.
// Texture coords are used to determine which corner
// of the billboard we compute.

uniform mat4 worldviewproj_matrix;
uniform vec4 size;
uniform vec4 normal;
uniform vec4 up;

#ifdef WITH_DEPTH
//include:
void passDepth( vec4 pos );
#endif

void main()
{
  vec3 right = cross(up.xyz, normal.xyz);
  
  vec4 s = gl_MultiTexCoord0 * size;
  vec3 r = s.xxx * right;
  vec3 u = s.yyy * up.xyz;
  
  vec4 pos = gl_Vertex + vec4( r + u, 0.0 );
  
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = gl_MultiTexCoord0 + vec4(0.5,0.5,0.0,0.0);
  gl_FrontColor = gl_Color;

#ifdef WITH_DEPTH
  passDepth( pos );
#endif
}
