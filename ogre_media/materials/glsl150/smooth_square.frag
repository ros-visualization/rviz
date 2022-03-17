#version 330

// rasterizes a smooth square with ax,ay in [-0.5..0.5]

uniform vec4 highlight;
uniform float alpha;

in vec4 texCoord[];
in vec4 frontColor;

out vec4 fragColor;

void main()
{
  float ax = texCoord[0].x-0.5;
  float ay = texCoord[0].y-0.5;

  float blend = smoothstep(-0.48, -0.45, ay) * (1.0 - smoothstep(0.45, 0.48, ay)) *
                smoothstep(-0.48, -0.45, ax) * (1.0 - smoothstep(0.45, 0.48, ax));

  float inv_blend = 1.0 - blend;
  vec3 col = blend * frontColor.xyz + (sign(0.5 - length(vec3(frontColor.xyz))) * vec3(0.2, 0.2, 0.2) + frontColor.xyz) * inv_blend;

  //alternative version: make color at edge darker
  //vec3 col = (0.5 + 0.5*blend) * gl_Color.xyz;

  col = col + col * highlight.xyz;

  fragColor = vec4(col.r, col.g, col.b, alpha * frontColor.a );
}
