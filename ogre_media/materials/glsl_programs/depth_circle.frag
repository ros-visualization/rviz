#version 120

// Draws a circle with the packed depth value 

// includes
vec4 packDepth( );
void circleImpl( vec4 color, float ax, float ay );

void main()
{
  circleImpl( packDepth(), gl_TexCoord[0].x-0.5, gl_TexCoord[0].y-0.5 );
}
