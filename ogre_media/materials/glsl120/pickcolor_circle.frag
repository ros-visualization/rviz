#version 120

// Draws a circle in the pick color 

//includes:
void circleImpl( vec4 color, float ax, float ay );

uniform vec4 pick_color;

void main()
{
  circleImpl( pick_color, gl_TexCoord[0].x-0.5, gl_TexCoord[0].y-0.5 );
}
