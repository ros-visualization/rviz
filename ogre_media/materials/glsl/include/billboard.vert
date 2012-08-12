#version 400

// Computes the position of a billboard vertex so
// the resulting quad will face the camera.
// Texture coords are used to determine which corner
// of the billboard we compute.

uniform mat4 worldviewproj_matrix;
uniform vec4 camera_pos;
uniform vec4 size;

in vec4 gl_Vertex;
in vec4 gl_MultiTexCoord0;

vec4 billboard()
{
  vec3 at = camera_pos.xyz - gl_Vertex.xyz;
  at = normalize(at);
  vec3 right = cross(vec3( 0.0, 1.0, 0.0 ), at);
  vec3 up = cross(at, right);
  right = normalize(right);
  up = normalize(up);
  
  vec4 s = gl_MultiTexCoord0 * size;
  vec3 r = s.xxx * right;
  vec3 u = s.yyy * up;
  
  vec4 dir = vec4( r + u, 0.0 );
  
  return worldviewproj_matrix * (gl_Vertex + dir);
}
