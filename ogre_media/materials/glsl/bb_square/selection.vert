#version 400


uniform vec4 pick_color;

out vec4 gl_Position;
out vec4 gl_FrontColor;

vec4 billboard();

void main()
{
  gl_Position = billboard();
  gl_FrontColor = vec4( pick_color.xyz, 1.0 );
}
