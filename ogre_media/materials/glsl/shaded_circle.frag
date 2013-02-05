#version 120

// rasterizes a circle that is darker at the borders than in the center

uniform vec4 highlight;
uniform float alpha;


void main()
{
  float ax = gl_TexCoord[0].x-0.5;
  float ay = gl_TexCoord[0].y-0.5;

  float rsquared = ax*ax+ay*ay;
  float a = (0.25 - rsquared) * 4.0;

  vec3 col = mix(vec3(0.8, 0.8, 0.8) * gl_Color.xyz, gl_Color.xyz, a);

  col = col + col * highlight.xyz;
  
  gl_FragColor = vec4(col, alpha * ceil(a) * gl_Color.a);
}
