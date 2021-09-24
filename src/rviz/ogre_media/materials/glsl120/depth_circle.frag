#version 120

// Draws a circle with the packed depth value 

// includes
vec4 packDepth( );
void circleImpl( vec4 color, float ax, float ay );

uniform float alpha;
uniform float far_clip_distance;

#include <pack_depth.frag>
#include <circle_impl.frag>

void main()
{
  circleImpl( packDepth(), gl_TexCoord[0].x-0.5, gl_TexCoord[0].y-0.5 );
}
