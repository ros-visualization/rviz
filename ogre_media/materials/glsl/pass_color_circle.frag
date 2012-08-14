#version 120

// Draws a circle in the fragment color

//includes:
void circleImpl( vec4 color, float ax, float ay );

void main()
{
  circleImpl( gl_Color, gl_TexCoord[0].x-0.5, gl_TexCoord[0].y-0.5 );
}
