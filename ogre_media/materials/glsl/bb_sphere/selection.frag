// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/point_cloud_billboard_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program pointCloudBillboardSphereFP_Selection_main
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 : TEX0 : 1 : 1
//var float4 pointCloudBillboardSphereFP_Selection_main.color : $vout.COLOR : COL : -1 : 1

struct Output {
    vec4 _color2;
};

float _TMP1;
float _x0004;

 // main procedure, the original name was pointCloudBillboardSphereFP_Selection_main
void main()
{

    float _rsquared;
    float _a;
    Output _OUT;

    _rsquared = gl_TexCoord[0].x*gl_TexCoord[0].x + gl_TexCoord[0].y*gl_TexCoord[0].y;
    _a = (2.50000000E-01 - _rsquared)*4.00000000E+00;
    _x0004 = -_a;
    _TMP1 = -floor(_x0004);
    _OUT._color2 = vec4(gl_Color.x, gl_Color.y, gl_Color.z, gl_Color.w*_TMP1);
    gl_FragColor = _OUT._color2;
    return;
} // main end
