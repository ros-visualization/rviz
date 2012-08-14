#version 120

// this merely passes over position and color, 
// as needed by box.geom

void main() {
    gl_Position = gl_Vertex;
    gl_FrontColor = gl_Color;
}
