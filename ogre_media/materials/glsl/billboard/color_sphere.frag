#version 120

//includes:
void shadedCircle_frag( float ax, float ay );

//in vec4 gl_TexCoord[];

void main()
{
  shadedCircle_frag( gl_TexCoord[0].x, gl_TexCoord[0].y );
}
