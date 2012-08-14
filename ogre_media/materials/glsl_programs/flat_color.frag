#version 120

// Passes the fragment color, replacing a with the alpha param

uniform vec4 highlight;
uniform float alpha;


void main()
{
  vec3 col = gl_Color.xyz + gl_Color.xyz * highlight.xyz;
  gl_FragColor = vec4(col, alpha);
}
