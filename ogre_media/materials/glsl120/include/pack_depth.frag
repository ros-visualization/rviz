#version 120

// Splits up a normalized depth value in the range (0..1) 
// into the vertex RGB values.
// Alpha values below 1/255 are rendered transparent.

uniform float alpha;
uniform float far_clip_distance;

const float minimum_alpha = 1.0 / 255.0;

varying float depth;

vec4 packDepth()
{
  float normalized_depth = depth / far_clip_distance;
  
  // split up float into rgb components
  const vec3 shift = vec3(256.0 * 256.0, 256.0, 1.0);
  const vec3 mask = vec3(0.0, 1.0 / 256.0, 1.0 / 256.0);
  vec3 depth_packed = fract(normalized_depth * shift);
  depth_packed -= depth_packed.xxy * mask;
  
  return vec4( depth_packed.zyx, step( minimum_alpha, alpha ));
}

