#version 120

// Computes the position of a billboard vertex so
// the resulting quad will face the camera.
// Texture coords are used to determine which corner
// of the billboard we compute.

uniform mat4 worldviewproj_matrix;
uniform vec4 camera_pos;
uniform vec4 size;
uniform vec4 up;
uniform vec4 normal;

void billboard_common_facing_vert()
{
  vec3 right = cross(up.xyz, normal.xyz);
  
  vec4 s = gl_MultiTexCoord0 * size;
  vec3 r = s.xxx * right;
  vec3 u = s.yyy * up.xyz;
  
  vec4 dir = vec4( r + u, 0.0 );
  
  gl_Position = worldviewproj_matrix * (gl_Vertex + dir);
  gl_TexCoord[0] = gl_MultiTexCoord0;
}
