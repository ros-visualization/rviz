// glslf output by Cg compiler
// cgc version 3.0.0007, build date Jul 22 2010
// command line args: -profile glslf
// source file: cg/point_cloud_box_fp.cg
//vendor NVIDIA Corporation
//version 3.0.0.07
//profile glslf
//program pointCloudBoxFP_Selection_main
//var float4 color : $vin.COLOR : COL0 : 0 : 1
//var float4 pointCloudBoxFP_Selection_main.color : $vout.COLOR : COL : -1 : 1

struct Output {
    vec4 _color2;
};


 // main procedure, the original name was pointCloudBoxFP_Selection_main
void main()
{

    Output _OUT;

    _OUT._color2 = vec4(gl_Color.x, gl_Color.y, gl_Color.z, 1.00000000E+00);
    gl_FragColor = _OUT._color2;
    return;
} // main end
