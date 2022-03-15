#version 120

// rasterizes a smooth square with ax,ay in [-0.5..0.5]

uniform vec4 highlight;
uniform float alpha;

void main()
{
  float ax = gl_TexCoord[0].x-0.5;
  float ay = gl_TexCoord[0].y-0.5;

  float blend = smoothstep(-0.48, -0.45, ay) * (1.0 - smoothstep(0.45, 0.48, ay)) *
                smoothstep(-0.48, -0.45, ax) * (1.0 - smoothstep(0.45, 0.48, ax));

  float inv_blend = 1.0 - blend;
  vec3 col = blend * gl_Color.rgb + (sign(0.5 - length(vec3(gl_Color.rgb))) * vec3(0.2, 0.2, 0.2) + gl_Color.rgb) * inv_blend;

  //alternative version: make color at edge darker
  //vec3 col = (0.5 + 0.5*blend) * gl_Color.rgb;

  col = col + col * highlight.rgb;

  gl_FragColor = vec4(col.rgb, alpha * gl_Color.a);
}
