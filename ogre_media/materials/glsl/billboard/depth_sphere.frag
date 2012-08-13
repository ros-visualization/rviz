#version 120

//includes:
void circle_frag( float ax, float ay );

void main()
{
  circle_frag( gl_TexCoord[0].x, gl_TexCoord[0].y );
}
