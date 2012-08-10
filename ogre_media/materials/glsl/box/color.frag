// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/point_cloud_box_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program pointCloudBoxFP_main
//semantic pointCloudBoxFP_main.highlight
//var float4 highlight :  : highlight : 2 : 1
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 : TEX0 : 1 : 1
//var float4 pointCloudBoxFP_main.color : $vout.COLOR : COL : -1 : 1

struct Output {
    vec4 _color2;
};

uniform vec4 highlight;
float _x0007;
float _TMP8;
float _b0013;
float _x0015;
float _TMP16;
float _b0021;
float _x0027;
float _TMP28;
float _b0033;
float _x0035;
float _TMP36;
float _b0041;
float _x0047;
float _TMP48;
float _b0053;
float _x0055;
float _TMP56;
float _b0061;
float _val0065;
float _a0065;
float _TMP68;
float _a0069;

 // main procedure, the original name was pointCloudBoxFP_main
void main()
{

    float _ax;
    float _ay;
    float _az;
    float _xstep;
    float _ystep;
    float _zstep;
    float _blend;
    float _inv_blend;
    vec3 _col;
    vec3 _factor;
    Output _OUT;

    _ax = 5.00000000E-01 - gl_TexCoord[0].x;
    _ay = 5.00000000E-01 - gl_TexCoord[0].y;
    _az = 5.00000000E-01 - gl_TexCoord[0].z;
    _x0007 = (_ax - 1.99999996E-02)/3.00000012E-02;
    _b0013 = min(1.00000000E+00, _x0007);
    _TMP8 = max(0.00000000E+00, _b0013);
    _x0015 = (_ax - 9.49999988E-01)/3.00000310E-02;
    _b0021 = min(1.00000000E+00, _x0015);
    _TMP16 = max(0.00000000E+00, _b0021);
    _xstep = _TMP8*_TMP8*(3.00000000E+00 - 2.00000000E+00*_TMP8)*(1.00000000E+00 - _TMP16*_TMP16*(3.00000000E+00 - 2.00000000E+00*_TMP16));
    _x0027 = (_ay - 1.99999996E-02)/3.00000012E-02;
    _b0033 = min(1.00000000E+00, _x0027);
    _TMP28 = max(0.00000000E+00, _b0033);
    _x0035 = (_ay - 9.49999988E-01)/3.00000310E-02;
    _b0041 = min(1.00000000E+00, _x0035);
    _TMP36 = max(0.00000000E+00, _b0041);
    _ystep = _TMP28*_TMP28*(3.00000000E+00 - 2.00000000E+00*_TMP28)*(1.00000000E+00 - _TMP36*_TMP36*(3.00000000E+00 - 2.00000000E+00*_TMP36));
    _x0047 = (_az - 1.99999996E-02)/3.00000012E-02;
    _b0053 = min(1.00000000E+00, _x0047);
    _TMP48 = max(0.00000000E+00, _b0053);
    _x0055 = (_az - 9.49999988E-01)/3.00000310E-02;
    _b0061 = min(1.00000000E+00, _x0055);
    _TMP56 = max(0.00000000E+00, _b0061);
    _zstep = _TMP48*_TMP48*(3.00000000E+00 - 2.00000000E+00*_TMP48)*(1.00000000E+00 - _TMP56*_TMP56*(3.00000000E+00 - 2.00000000E+00*_TMP56));
    _blend = _xstep*_ystep + _xstep*_zstep + _ystep*_zstep;
    _inv_blend = 1.00000000E+00 - _blend;
    _a0069 = dot(gl_Color.xyz, gl_Color.xyz);
    _TMP68 = 1.00000000E+00/inversesqrt(_a0069);
    _a0065 = 5.00000000E-01 - _TMP68;
    _val0065 = float((_a0065 > 0.00000000E+00));
    _col = _blend*gl_Color.xyz + ((_val0065 - float((_a0065 < 0.00000000E+00)))*vec3( 2.00000003E-01, 2.00000003E-01, 2.00000003E-01) + gl_Color.xyz)*_inv_blend;
    _col = _col + _col*highlight.xyz;
    _factor = vec3( 1.00000000E+00, 1.00000000E+00, 1.00000000E+00);
    if (gl_TexCoord[0].z < -4.99900013E-01) { // if begin
        _factor = vec3( 1.00000000E+00, 1.00000000E+00, 1.00000000E+00);
    } else {
        if (gl_TexCoord[0].x > 4.99900013E-01) { // if begin
            _factor = vec3( 8.99999976E-01, 8.99999976E-01, 8.99999976E-01);
        } else {
            if (gl_TexCoord[0].x < -4.99900013E-01) { // if begin
                _factor = vec3( 5.00000000E-01, 5.00000000E-01, 5.00000000E-01);
            } else {
                if (gl_TexCoord[0].y > 4.99900013E-01) { // if begin
                    _factor = vec3( 6.00000024E-01, 6.00000024E-01, 6.00000024E-01);
                } else {
                    if (gl_TexCoord[0].y < -4.99900013E-01) { // if begin
                        _factor = vec3( 6.00000024E-01, 6.00000024E-01, 6.00000024E-01);
                    } else {
                        if (gl_TexCoord[0].z > 4.99900013E-01) { // if begin
                            _factor = vec3( 4.00000006E-01, 4.00000006E-01, 4.00000006E-01);
                        } // end if
                    } // end if
                } // end if
            } // end if
        } // end if
    } // end if
    _col = _col.xyz*_factor;
    _OUT._color2 = vec4(_col.x, _col.y, _col.z, gl_Color.w);
    gl_FragColor = _OUT._color2;
    return;
} // main end
