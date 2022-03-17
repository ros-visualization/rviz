#version 330

// this merely passes over position and color,
// as needed by box.geom

layout(location = 0) in vec3 vertex;
layout(location = 3) in vec4 color;

uniform mat4 uProjection;

out PerVertex {
    vec4 frontColor;
} perVertex;


void main() {
    gl_Position = uProjection * vec4(vertex, 1);
    perVertex.frontColor = color;
}
