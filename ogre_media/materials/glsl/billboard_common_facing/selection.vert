#version 120

//includes:
void billboard_common_facing_vert();

uniform vec4 pick_color;

void main()
{
  billboard_common_facing_vert();
  gl_FrontColor = vec4( pick_color.xyz, 1.0 );
}
