#version 120

uniform vec4 highlight;
//in vec4 gl_Color;
//out vec4 gl_FragColor;

// rasterizes a smooth square with ax,ay in [0..1]
void smoothSquare_frag( float ax, float ay )
{
  float blend = smoothstep(0.02, 0.05, ay) * (1.0 - smoothstep(0.95, 0.98, ay)) *
                smoothstep(0.02, 0.05, ax) * (1.0 - smoothstep(0.95, 0.98, ax));
  float inv_blend = 1.0 - blend;
  vec3 col = blend * gl_Color.xyz + (sign(0.5 - length(vec3(gl_Color.xyz))) * vec3(0.2, 0.2, 0.2) + gl_Color.xyz) * inv_blend;

  col = col + col * highlight.xyz;

  gl_FragColor = vec4(col.r, col.g, col.b, gl_Color.a);
}
