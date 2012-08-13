#version 120

//includes:
void billboard_common_facing_vert();
void passDepth_vert();

void main()
{
  billboard_common_facing_vert();
  passDepth_vert();
}
