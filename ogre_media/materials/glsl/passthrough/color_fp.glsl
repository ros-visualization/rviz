// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/passthrough.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program passthroughFP_main
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float4 passthroughFP_main.color : $vout.COLOR : COL : -1 : 1

struct VPOutput {
    vec4 _color2;
    vec4 _tex;
};

struct FPOutput {
    vec4 _color3;
};


 // main procedure, the original name was passthroughFP_main
void main()
{


    gl_FragColor = gl_Color;
    return;
} // main end
