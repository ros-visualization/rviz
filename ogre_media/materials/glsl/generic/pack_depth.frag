// pack floating point depth value into
// the components the rgb output

// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/depth.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program depthFP_main
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 :  : 1 : 0
//var float4 depthFP_main.color : $vout.COLOR : COL : -1 : 1

struct VPOutput {
    vec4 _color2;
    vec4 _tex2;
};

struct FPOutput {
    vec4 _color3;
};

vec3 _ret0002;
vec3 _x0004;

 // main procedure, the original name was depthFP_main
void main()
{

    FPOutput _OUT;

    _x0004 = gl_Color.x*vec3( 6.55360000E+04, 2.56000000E+02, 1.00000000E+00);
    _ret0002 = fract(_x0004);
    _ret0002 = _ret0002 - _ret0002.xxy*vec3( 0.00000000E+00, 3.90625000E-03, 3.90625000E-03);
    _OUT._color3 = vec4(_ret0002.z, _ret0002.y, _ret0002.x, gl_Color.w);
    gl_FragColor = _OUT._color3;
    return;
} // main end
