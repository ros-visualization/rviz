#version 120

void smoothSquare_frag( float ax, float ay );

void main()
{
  smoothSquare_frag( gl_PointCoord.x, gl_PointCoord.y );
}
