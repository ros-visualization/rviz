#version 400

out vec4 gl_FrontColor;

uniform float4 pick_color;

void pointSprite();
 
void main()
{
  pointSprite();
  gl_FrontColor = vec4( pick_color, 1.0 );
}
