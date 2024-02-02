#version 150

// Generates an axis-aligned box with a given size
// for each input vertex.

#ifdef WITH_DEPTH
  uniform mat4 worldview_matrix;
  out float depth;

  void passDepth( vec4 pos )
  {
    vec4 pos_rel_view = worldview_matrix * pos;
    depth = -pos_rel_view.z;
  }
#endif


uniform mat4 worldviewproj_matrix;
uniform vec4 size;
uniform vec4 auto_size;

in VertexData {
	vec4 color;
} vdata[];

// Use exactly these names to map onto gl_Color and gl_TexCoord used by fragment shaders
out vec4 color;
out vec2 TexCoord;

layout(points) in;
layout(triangle_strip, max_vertices=24) out;


const vec4 axes[3] = vec4[] (
  vec4(1.0,0.0,0.0,0.0),
  vec4(0.0,1.0,0.0,0.0),
  vec4(0.0,0.0,1.0,0.0)	);

const float lightness[6] = float[] (
    0.9, 0.5, 0.6, 0.6, 1.0, 0.4 );

void emitVertex( int side, vec4 x, vec4 y, vec4 z, vec3 tex, vec4 size_factor )
{
  vec4 pos_rel = tex.x*x + tex.y*y + tex.z*z;
  vec4 pos = gl_in[0].gl_Position + vec4( pos_rel * size_factor );
  gl_Position = worldviewproj_matrix * pos;
  TexCoord = vec2(tex.x*0.5+0.5, tex.y*0.5+0.5);

#ifdef WITH_LIGHTING
    color = vec4( vdata[0].color.rgb * lightness[side], vdata[0].color.a );
#else
    color = vdata[0].color;
#endif

#ifdef WITH_DEPTH
  passDepth( pos );
#endif

  EmitVertex();
}


void main()
{
  // if auto_size == 1, then size_factor == size*gl_Vertex.z
  // if auto_size == 0, then size_factor == size
  vec4 size_factor = (0.5*(1-auto_size.x+(auto_size.x*gl_in[0].gl_Position.z)))*size;

  for( int side=0; side<3; side++ )
  {
    vec4 x=axes[ side ];
    vec4 y=axes[ (side+1)%3 ];
    vec4 z=axes[ (side+2)%3 ];

    // face for +z
    emitVertex( side, x, y, z, vec3(-1, -1, +1), size_factor );
    emitVertex( side, x, y, z, vec3(+1, -1, +1), size_factor );
    emitVertex( side, x, y, z, vec3(-1, +1, +1), size_factor );
    emitVertex( side, x, y, z, vec3(+1, +1, +1), size_factor );
    EndPrimitive();

    // face for -z
    emitVertex( side+3, x, y, z, vec3(-1, -1, -1), size_factor );
    emitVertex( side+3, x, y, z, vec3(-1, +1, -1), size_factor );
    emitVertex( side+3, x, y, z, vec3(+1, -1, -1), size_factor );
    emitVertex( side+3, x, y, z, vec3(+1, +1, -1), size_factor );
    EndPrimitive();
  }
}
