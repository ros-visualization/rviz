#version 150

// need to use exactly these names to have OGRE bind the unpacked values:
// https://ogrecave.github.io/ogre/api/latest/_high-level-_programs.html#Binding-vertex-attributes
in vec4 vertex;
in vec4 colour;

// this merely passes over position and color as needed by box.geom

out VertexData {
    vec4 color;
} vdata;

void main() {
    gl_Position = vertex;
    vdata.color = colour;
}
