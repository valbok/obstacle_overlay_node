#version 120

// Minimal depth passthrough shader

uniform mat4 worldviewproj_matrix;
uniform mat4 worldview_matrix;

void passDepth( vec4 pos );

#include <pass_depth.vert>

void main() {
    gl_Position = worldviewproj_matrix * gl_Vertex;
    passDepth( gl_Vertex );
}
