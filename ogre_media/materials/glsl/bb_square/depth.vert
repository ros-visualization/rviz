#version 400

out vec4 gl_Position;

in vec4 gl_Color;

vec4 billboard();
void passDepth();

void main()
{
  vec4 vertex_pos = billboard();
  gl_Position = billboard();
  passDepth();
}
