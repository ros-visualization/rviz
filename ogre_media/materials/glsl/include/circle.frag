#version 120

//in vec4 gl_Color;
//out vec4 gl_FragColor;

// rasterizes a circle of radius 0.5
void circle_frag( float ax, float ay )
{
  float rsquared = ax*ax+ay*ay;
  float a = (0.25 - rsquared) * 4.0;
  gl_FragColor = vec4(gl_Color.rgb, gl_Color.a * ceil(a));
}
