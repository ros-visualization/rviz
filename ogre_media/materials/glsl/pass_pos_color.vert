#version 150

// this merely passes over position and color, 
// as needed by box.geom

out gl_PerVertex {
	vec4 gl_Position;
	vec4 gl_FrontColor;
	//vec4 gl_TexCoord[];
};

in vec4 gl_Vertex;
in vec4 gl_Color;

void main() {
    gl_Position = gl_Vertex;
    gl_FrontColor = gl_Color;
}
