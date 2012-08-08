#version 120

uniform vec4 highlight;

vec4 getColor( vec4 color, vec4 highlight, float dist )
{
  float mult = 1.0 - dist*dist;
  color = color + color*highlight;
  return color * mult;
}

// paints a box-shaped point sprite
void main()
{
  float dist = max( abs(gl_PointCoord.x-0.5), abs(gl_PointCoord.y-0.5) );
  gl_FragColor = getColor( gl_Color, highlight, dist );
}
