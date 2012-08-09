// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/point_cloud_billboard_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program pointCloudBillboardSphereFP_depth
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 : TEX0 : 1 : 1
//var float4 pointCloudBillboardSphereFP_depth.color : $vout.COLOR : COL : -1 : 1

struct Output {
    vec4 _color2;
};

vec3 _ret0002;
vec3 _x0004;
float _TMP5;
float _x0008;

 // main procedure, the original name was pointCloudBillboardSphereFP_depth
void main()
{

    float _rsquared;
    float _a;
    Output _OUT;

    _rsquared = gl_TexCoord[0].x*gl_TexCoord[0].x + gl_TexCoord[0].y*gl_TexCoord[0].y;
    _a = (2.50000000E-01 - _rsquared)*4.00000000E+00;
    _x0004 = gl_Color.x*vec3( 6.55360000E+04, 2.56000000E+02, 1.00000000E+00);
    _ret0002 = fract(_x0004);
    _ret0002 = _ret0002 - _ret0002.xxy*vec3( 0.00000000E+00, 3.90625000E-03, 3.90625000E-03);
    _x0008 = -_a;
    _TMP5 = -floor(_x0008);
    _OUT._color2 = vec4(_ret0002.z, _ret0002.y, _ret0002.x, gl_Color.w*_TMP5);
    gl_FragColor = _OUT._color2;
    return;
} // main end
