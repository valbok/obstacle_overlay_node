#version 120

// Passes on the packed depth value

// includes
vec4 packDepth( );
uniform float alpha;
uniform float far_clip_distance;

#include <pack_depth.frag>

void main()
{
  gl_FragColor = packDepth();
}
