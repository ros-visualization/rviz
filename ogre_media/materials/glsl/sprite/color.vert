#version 120

//in vec4 gl_Color;
//out vec4 gl_FrontColor;

uniform vec4 alpha;

void pointSprite_vert();
 
void main()
{
  pointSprite_vert();
  gl_FrontColor = vec4( gl_Color.rgb, alpha.x );
}
