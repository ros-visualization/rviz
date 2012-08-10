#version 400

in vec4 gl_Color;
out vec4 gl_FrontColor;
uniform vec4 alpha;

void pointSprite();
 
void main()
{
  pointSprite();
  gl_FrontColor = vec4( gl_Color.xyz, alpha.x);
}
