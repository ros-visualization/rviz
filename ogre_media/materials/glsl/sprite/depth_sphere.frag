#version 120

//in vec2 gl_PointCoord;

// rasterizes a circle of radius 0.5
void circle_frag( float ax, float ay );

void main()
{
  circle_frag( gl_PointCoord.x-0.5, gl_PointCoord.y-0.5 );
}
