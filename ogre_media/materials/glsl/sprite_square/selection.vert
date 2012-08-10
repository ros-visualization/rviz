#version 400

out vec4 gl_FrontColor;

uniform vec4 pick_color;

void pointSprite();

void main()
{
  pointSprite();
  gl_FrontColor = vec4( pick_color.xyz, 1.0 );
}
