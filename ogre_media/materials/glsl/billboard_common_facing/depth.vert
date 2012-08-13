#version 120

//includes:
void billboard_common_facing_vert();
void packDepth_vert();

void main()
{
  billboard_common_facing_vert();
  packDepth_vert();
}
