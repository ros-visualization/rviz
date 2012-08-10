#version 400

// Passes the z value in camera coordinates 
// into the vertex color.

uniform mat4 worldview_matrix;
uniform float far_clip_distance;
uniform float alpha;

in vec4 gl_Vertex;
out vec4 gl_FrontColor;

void passDepth()
{
  vec4 pos_rel_view = worldview_matrix * gl_Vertex;
  float normalized_depth = -pos_rel_view.z / far_clip_distance;

  float minimum_alpha = 1.0 / 255;

  gl_FrontColor = vec4( normalized_depth, normalized_depth, normalized_depth, step( minimum_alpha, alpha ));
}
