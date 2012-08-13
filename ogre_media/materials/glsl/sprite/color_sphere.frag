#version 120

//in vec2 gl_PointCoord;

void shadedCircle_frag( float ax, float ay );

void main()
{
  shadedCircle_frag( gl_PointCoord.x-0.5, gl_PointCoord.y-0.5 );
}
