#version 150

// rasterizes a smooth square with ax,ay in [-0.5..0.5]

uniform vec4 highlight;
uniform float alpha;

in vec4 color;
in vec2 TexCoord;

out vec4 FragColor;

void main()
{
  const float outer = 0.48;
  const float inner = 0.45;
  float ax = TexCoord.x-0.5;
  float ay = TexCoord.y-0.5;

  // blending factor, varying between 0.0 (at the border) and 1.0 (at the center of the square)
  float blend = smoothstep(-outer, -inner, ay) * (1.0 - smoothstep(inner, outer, ay)) *
                smoothstep(-outer, -inner, ax) * (1.0 - smoothstep(inner, outer, ax));

#if 1 // high contrast edge (dark edge on light surface / light edge on dark surface)
  float inv_blend = 1.0 - blend;
  vec3 col = blend * color.rgb + (sign(0.5 - length(vec3(color.rgb))) * vec3(0.2, 0.2, 0.2) + color.rgb) * inv_blend;
#else // alternatively: make color at edge darker
  vec3 col = (0.5 + 0.5*blend) * color.rgb;
#endif

  col = col + col * highlight.rgb;

  FragColor = vec4(col.rgb, alpha * color.a);
}
