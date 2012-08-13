#version 120

//includes:
void billboard_common_facing_vert();

uniform vec4 alpha;

void main()
{
  billboard_common_facing_vert();
  gl_FrontColor = vec4( gl_Color.rgb, alpha.x);
}
