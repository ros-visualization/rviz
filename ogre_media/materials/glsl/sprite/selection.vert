#version 120

//out vec4 gl_FrontColor;

uniform vec4 pick_color;

void pointSprite_vert();

void main()
{
  pointSprite_vert();
  gl_FrontColor = vec4( pick_color.xyz, 1.0 );
}
