#version 150

// rasterizes a circle that is darker at the borders than in the center

uniform vec4 highlight;
uniform float alpha;

in vec4 color;
in vec2 TexCoord;

out vec4 FragColor;

void main()
{
  float ax = TexCoord.x-0.5;
  float ay = TexCoord.y-0.5;

  float rsquared = ax*ax+ay*ay;
  float a = (0.25 - rsquared) * 4.0;

  vec3 col = mix(vec3(0.8, 0.8, 0.8) * color.xyz, color.xyz, a);

  col = col + col * highlight.xyz;

  FragColor = vec4(col, alpha * ceil(a) * color.a);
}
