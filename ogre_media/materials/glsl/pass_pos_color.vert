#version 150 compatibility

// this merely passes over position and color, 
// as needed by box.geom

out gl_PerVertex {
	vec4 gl_Position;
	vec4 gl_FrontColor;
};

void main() {
    gl_Position = gl_Vertex;
    gl_FrontColor = gl_Color;
}
