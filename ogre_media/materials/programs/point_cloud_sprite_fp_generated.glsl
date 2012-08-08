point_cloud_sprite_fp.cg
// glslv output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslv
// source file: point_cloud_sprite_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslv
//program pointCloudSpriteFP_main
//semantic pointCloudSpriteFP_main.highlight
//var float4 highlight :  : _highlight1 : 2 : 1
//var float4 color : $vin.COLOR : ATTR3 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 : ATTR8 : 1 : 1
//var float4 pointCloudSpriteFP_main.color : $vout.COLOR : COL0 : -1 : 1

struct Output {
    vec4 _color2;
};

uniform vec4 highlight;
float _x0011;
float _TMP12;
float _b0017;
float _x0019;
float _TMP20;
float _b0025;
float _x0027;
float _TMP28;
float _b0033;
float _x0035;
float _TMP36;
float _b0041;
float _val0045;
float _a0045;
float _TMP48;
float _a0049;

 // main procedure, the original name was pointCloudSpriteFP_main
void main()
{

    float _blend;
    float _inv_blend;
    vec3 _col;
    Output _OUT;

    _x0011 = (gl_PointCoord.y - 1.99999996E-02)/3.00000012E-02;
    _b0017 = min(1.00000000E+00, _x0011);
    _TMP12 = max(0.00000000E+00, _b0017);
    _x0019 = (gl_PointCoord.y - 9.49999988E-01)/3.00000310E-02;
    _b0025 = min(1.00000000E+00, _x0019);
    _TMP20 = max(0.00000000E+00, _b0025);
    _x0027 = (gl_PointCoord.x - 1.99999996E-02)/3.00000012E-02;
    _b0033 = min(1.00000000E+00, _x0027);
    _TMP28 = max(0.00000000E+00, _b0033);
    _x0035 = (gl_PointCoord.x - 9.49999988E-01)/3.00000310E-02;
    _b0041 = min(1.00000000E+00, _x0035);
    _TMP36 = max(0.00000000E+00, _b0041);
    _blend = _TMP12*_TMP12*(3.00000000E+00 - 2.00000000E+00*_TMP12)*(1.00000000E+00 - _TMP20*_TMP20*(3.00000000E+00 - 2.00000000E+00*_TMP20))*_TMP28*_TMP28*(3.00000000E+00 - 2.00000000E+00*_TMP28)*(1.00000000E+00 - _TMP36*_TMP36*(3.00000000E+00 - 2.00000000E+00*_TMP36));
    _inv_blend = 1.00000000E+00 - _blend;
    _a0049 = dot(gl_Color.xyz, gl_Color.xyz);
    _TMP48 = 1.00000000E+00/inversesqrt(_a0049);
    _a0045 = 5.00000000E-01 - _TMP48;
    _val0045 = float((_a0045 > 0.00000000E+00));
    _col = _blend*gl_Color.xyz + ((_val0045 - float((_a0045 < 0.00000000E+00)))*vec3( 2.00000003E-01, 2.00000003E-01, 2.00000003E-01) + gl_Color.xyz)*_inv_blend;
    _col = _col + _col*_highlight1.xyz;
    _OUT._color2 = vec4(_col.x, _col.y, _col.z, gl_Color.w);
    gl_FragColor = _OUT._color2;
    return;
} // main end
