#version 120

// Generic vertex shader for point sprites
// sets position and point size.
// Works for perspective and orthographic projection.

uniform mat4 worldviewproj_matrix;
uniform mat4 worldview_matrix;
uniform mat4 projection_matrix;
uniform float viewport_height_pixels;
uniform vec4 size;

#ifdef WITH_DEPTH
  //include:
  void passDepth( vec4 pos );
#endif

void main()
{
  vec4 pos_rel_cam = worldview_matrix * gl_Vertex;
  float pixels_per_meter = viewport_height_pixels * abs( projection_matrix[1][1] ) * 0.5;

  // The following code does this for perspective:
  //   point_size = pixels_per_meter * size.x / -pos_rel_cam.z;
  // and does this for ortho:
  //   point_size = pixels_per_meter * size.x;
  // by using the fact that the lower-right part of projection_matrix 
  // is [-1 0] for perspective and [0 1] for ortho.

  vec2 distance_factor = vec2( 1.0 / pos_rel_cam.z, 1.0 );
  vec2 perspective_or_ortho;
  perspective_or_ortho.x = projection_matrix[2][3];
  perspective_or_ortho.y = projection_matrix[3][3];
  
  gl_Position = worldviewproj_matrix * gl_Vertex;
  gl_FrontColor = gl_Color;
  gl_PointSize = pixels_per_meter * size.x * dot( distance_factor, perspective_or_ortho );

#ifdef WITH_DEPTH
  passDepth( gl_Vertex );
#endif
}
