#version 150

// Generates view-facing square (billboard) with a given size
// for each input vertex.

#ifdef WITH_DEPTH
  uniform mat4 worldview_matrix;
  out float depth;
#endif

uniform mat4 inverse_worldview_matrix
uniform mat4 worldviewproj_matrix;
uniform vec4 camera_pos;
uniform vec4 size;
uniform vec4 auto_size;

in gl_PerVertex {
	vec4 gl_Position;
	vec4 gl_FrontColor;
} gl_in[];

out vec4 gl_TexCoord[];

layout(points) in;
layout(triangle_strip, max_vertices=4) out;

void emitVertex( vec3 pos_rel, vec3 tex )
{
  vec4 pos = gl_in[0].gl_Position + inverse_worldview_matrix * pos_rel;
  gl_Position = worldviewproj_matrix * pos;
  gl_TexCoord[0] = vec4( tex.x*0.5+0.5, tex.y*0.5+0.5, 0.0, 0.0 );
  gl_FrontColor = vec4( gl_in[0].gl_FrontColor );
  EmitVertex();
}

void main() 
{
  // if auto_size == 1, then size_factor == size.x*gl_Vertex.z
  // if auto_size == 0, then size_factor == size.x
  float size_factor = (1-auto_size.x+(auto_size.x*gl_in[0].gl_Position.z))*size.x;
  size_factor = size_factor * 0.5;

#ifdef WITH_DEPTH
  depth = (worldview_matrix * pos).z;
#endif

  emitVertex( vec3(-size,-size,0), vec3(0,0,0) );
  emitVertex( vec3(size,-size,0), vec3(1,0,0) );
  emitVertex( vec3(-size,size,0), vec3(-1,1,0) );
  emitVertex( vec3(size,size,0), vec3(1,1,0) );
  
  EndPrimitive();
}

