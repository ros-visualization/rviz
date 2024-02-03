#version 150

// Passes the fragment color, multiplying a with the alpha param

uniform vec4 highlight;
uniform float alpha;

in vec4 color;
out vec4 FragColor;

void main()
{
  vec3 col = color.xyz + color.xyz * highlight.xyz;
  FragColor = vec4(col, color.a * alpha);
}
