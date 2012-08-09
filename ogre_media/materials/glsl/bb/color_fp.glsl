#version 120

uniform vec4 highlight;

void smooth_rect( vec4 highlight, float ax, float ay );

void main()
{
  smooth_rect( highlight, gl_TexCoord[0].x + 0.5, gl_TexCoord[0].y + 0.5 );
}
