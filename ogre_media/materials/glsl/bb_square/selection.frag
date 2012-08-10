// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/point_cloud_billboard_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program pointCloudBillboardFP_selection
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float3 tex : $vin.TEXCOORD0 : TEX0 : 1 : 1
//var float4 pointCloudBillboardFP_selection.color : $vout.COLOR : COL : -1 : 1

struct Output {
    vec4 _color2;
};


 // main procedure, the original name was pointCloudBillboardFP_selection
void main()
{

    float _ax;
    float _ay;
    float _blend;
    vec3 _col;
    Output _OUT;

    _ax = 5.00000000E-01 - gl_TexCoord[0].x;
    _ay = 5.00000000E-01 - gl_TexCoord[0].y;
    _blend = float((_ay >= 3.50000001E-02))*(1.00000000E+00 - float((_ay >= 9.64999974E-01)))*float((_ax >= 3.50000001E-02))*(1.00000000E+00 - float((_ax >= 9.64999974E-01)));
    _col = _blend*gl_Color.xyz;
    _OUT._color2 = vec4(_col.x, _col.y, _col.z, _blend);
    gl_FragColor = _OUT._color2;
    return;
} // main end
