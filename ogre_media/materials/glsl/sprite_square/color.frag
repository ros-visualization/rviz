#version 400

void smoothSquare( float ax, float ay );

void main()
{
  smoothSquare( gl_PointCoord.x, gl_PointCoord.y );
}
