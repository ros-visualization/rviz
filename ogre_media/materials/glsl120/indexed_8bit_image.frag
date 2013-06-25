#version 120

// Draws an 8-bit image using RGB values from a 256x1 palette texture.

varying vec2 UV;
uniform sampler2D eight_bit_image;
uniform sampler1D palette;
uniform float alpha;

void main()
{
  // The 0.999 multiply is needed because brightness value 255 comes
  // out of texture2D() as 1.0, which wraps around to 0.0 in the
  // palette texture.
  vec4 color = texture1D( palette, 0.999 * texture2D( eight_bit_image, UV ).x );
  gl_FragColor = vec4( color.rgb, color.a * alpha );
}
