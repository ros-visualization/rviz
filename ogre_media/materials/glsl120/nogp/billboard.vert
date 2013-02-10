#version 120

// Computes the position of a billboard vertex so
// the resulting quad will face the camera.
// Texture coords are used to determine which corner
// of the billboard we compute.

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
  vec3 at = camera_pos.xyz - gl_Vertex.xyz;
  at = normalize(at);
  vec3 right = cross(vec3( 0.0, 1.0, 0.0 ), at);
  vec3 up = cross(at, right);
  right = normalize(right);
  up = normalize(up);

  // if auto_size == 1, then size_factor == size*gl_Vertex.z
  // if auto_size == 0, then size_factor == size
  vec4 size_factor = (1-auto_size.x+(auto_size.x*gl_Vertex.z))*size;

  vec4 s = gl_MultiTexCoord0 * (size_factor);
  vec3 r = s.xxx * right;
  vec3 u = s.yyy * up;
  
  vec4 pos = gl_Vertex + vec4( r + u, 0.0 );
  
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = gl_MultiTexCoord0 + vec4(0.5,0.5,0.0,0.0);
  gl_FrontColor = gl_Color;

#ifdef WITH_DEPTH
  passDepth( pos );
#endif
}
