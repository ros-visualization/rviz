#version 120

// rasterizes a smooth square on any face of a box
// and creates a fake lighting effect
// x,y,z texture coords must be in the interval (-0.5...0.5)

uniform vec4 highlight;
uniform float alpha;

const float lightness[6] = float[] (
    0.9, 0.5, 0.6, 0.6, 1.0, 0.4 );

void main()
{
  float ax;
  float ay;
  float l;
  
  if ( gl_TexCoord[0].z < -0.4999 )
  {
    ax = gl_TexCoord[0].x;
    ay = gl_TexCoord[0].y;
    l = lightness[0];
  }
  else if ( gl_TexCoord[0].z > 0.4999 )
  {
    ax = gl_TexCoord[0].x;
    ay = gl_TexCoord[0].y;
    l = lightness[1];
  }
  else if ( gl_TexCoord[0].x > 0.4999 )
  {
    ax = gl_TexCoord[0].y;
    ay = gl_TexCoord[0].z;
    l = lightness[2];
  }
  else if ( gl_TexCoord[0].x < -0.4999 )
  {
    ax = gl_TexCoord[0].y;
    ay = gl_TexCoord[0].z;
    l = lightness[3];
  }
  else if ( gl_TexCoord[0].y > 0.4999 )
  {
    ax = gl_TexCoord[0].x;
    ay = gl_TexCoord[0].z;
    l = lightness[4];
  }
  else
  {
    ax = gl_TexCoord[0].x;
    ay = gl_TexCoord[0].z;
    l = lightness[5];
  }

  float blend = smoothstep(-0.48, -0.45, ay) * (1.0 - smoothstep(0.45, 0.48, ay)) *
                smoothstep(-0.48, -0.45, ax) * (1.0 - smoothstep(0.45, 0.48, ax));
  
  float inv_blend = 1.0 - blend;
  
  //vec3 col = blend * gl_Color.xyz + (sign(0.5 - length(vec3(gl_Color.xyz))) * vec3(0.2, 0.2, 0.2) + gl_Color.xyz) * inv_blend;
  
  //alternative version: make color at edge darker
  vec3 col = (0.5 + 0.5*blend) * gl_Color.xyz;
  
  col = col * l;
  
  col = col + col * highlight.xyz;


  gl_FragColor = vec4(col.r, col.g, col.b, alpha * gl_Color.a );
}
