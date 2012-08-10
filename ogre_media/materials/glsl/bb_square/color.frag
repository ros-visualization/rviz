#version 400

//uniform vec4 highlight;
in vec4 gl_TexCoord[];

void smoothSquare( float ax, float ay );

void main()
{
  smoothSquare( gl_TexCoord[0].x + 0.5, gl_TexCoord[0].y + 0.5 );
}
