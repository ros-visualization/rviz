// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/point_cloud_billboard_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program pointCloudBillboardSphereFP_main
//semantic pointCloudBillboardSphereFP_main.highlight
//var float4 highlight :  : _highlight1 : 2 : 1
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 : TEX0 : 1 : 1
//var float4 pointCloudBillboardSphereFP_main.color : $vout.COLOR : COL : -1 : 1

struct Output {
    vec4 _color2;
};

uniform vec4 _highlight1;
vec3 _a0003;
float _t0003;
float _TMP4;
float _TMP6;
float _x0007;

 // main procedure, the original name was pointCloudBillboardSphereFP_main
void main()
{

    float _rsquared;
    float _a;
    vec3 _col;
    Output _OUT;

    _rsquared = float((gl_TexCoord[0].x*gl_TexCoord[0].x + gl_TexCoord[0].y*gl_TexCoord[0].y));
    _a = (2.50000000E-01 - _rsquared)*4.00000000E+00;
    _a0003 = vec3( 7.99804688E-01, 7.99804688E-01, 7.99804688E-01)*gl_Color.xyz;
    _t0003 = float(_a);
    _col = _a0003 + _t0003*(gl_Color.xyz - _a0003);
    _col = _col + _col*_highlight1.xyz;
    _x0007 = -_a;
    _TMP6 = float(floor(float(_x0007)));
    _TMP4 = -_TMP6;
    _OUT._color2 = vec4(_col.x, _col.y, _col.z, gl_Color.w*float(_TMP4));
    gl_FragColor = _OUT._color2;
    return;
} // main end
