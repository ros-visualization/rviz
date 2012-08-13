#version 120

uniform vec4 highlight;
//in vec4 gl_Color;
//out vec4 gl_FragColor;

// rasterizes a smooth square with ax,ay in [0..1]
void shadedCircle_frag( float ax, float ay )
{
  float rsquared = ax*ax+ay*ay;
  float a = (0.25 - rsquared) * 4.0;

  vec3 col = mix(vec3(0.8, 0.8, 0.8) * gl_Color.xyz, gl_Color.xyz, a);

  col = col + col * highlight.xyz;
  
  //gl_FragColor=vec4(ceil(a),ceil(a),ceil(a),1.0);
  gl_FragColor = vec4(col, gl_Color.a * ceil(a));
}
