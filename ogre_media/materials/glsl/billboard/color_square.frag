#version 120

//includes:
void smoothSquare_frag( float ax, float ay );

//in vec4 gl_TexCoord[];

void main()
{
  smoothSquare_frag( gl_TexCoord[0].x + 0.5, gl_TexCoord[0].y + 0.5 );
}
