#version 120

//includes:
void billboard_vert();

uniform vec4 pick_color;

//out vec4 gl_FrontColor;

void main()
{
  billboard_vert();
  gl_FrontColor = vec4( pick_color.xyz, 1.0 );
}
