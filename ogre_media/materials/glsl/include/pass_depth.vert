#version 120

// Passes the normalized depth unchanged into the vertex color.

uniform mat4 worldview_matrix;
uniform float alpha;
uniform float far_clip_distance;

//in vec4 gl_Vertex;
//out vec4 gl_FrontColor;

void passDepth_vert()
{
  vec4 pos_rel_view = worldview_matrix * gl_Vertex;
  float d = - pos_rel_view.z / far_clip_distance;
  gl_FrontColor = vec4( d, d, d, 1.0 );
}
