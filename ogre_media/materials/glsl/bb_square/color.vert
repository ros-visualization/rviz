#version 400

uniform vec4 alpha;

in vec4 gl_Color;

out vec4 gl_Position;
out vec4 gl_FrontColor;

vec4 billboard();

void main()
{
  gl_Position = billboard();
  gl_FrontColor = vec4( gl_Color.rgb, alpha.x);
}
