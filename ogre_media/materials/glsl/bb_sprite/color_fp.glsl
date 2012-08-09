uniform vec4 highlight;

void smooth_rect( vec4 highlight, float ax, float ay );

void main()
{
  smooth_rect( highlight, gl_PointCoord.x, gl_PointCoord.y );
  return;
}
