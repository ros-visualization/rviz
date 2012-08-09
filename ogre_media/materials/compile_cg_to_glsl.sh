#!/bin/bash

set -e

#Compiles all cg scripts into glsl 

compile () { 
  echo "------------------------------------------"
  echo "type   = $1"
  echo "in     = $2"
  echo "main() = $3"
  echo "out    = $4/$5"
  
  OUT_DIR="glsl/$4"
  
  if [ ! -d $OUT_DIR ] 
    then  
  	mkdir $OUT_DIR
  fi
  
  OUT_FILE="$OUT_DIR/$5_$1p.glsl"
  
  # compile
  cgc cg/$2 -entry $3 -o $OUT_FILE -profile glsl$1
  
  # fix names of input variables
  sed -i 's/_highlight1/highlight/g' $OUT_FILE
}

# fragment shaders

compile f point_cloud_billboard_fp.cg pointCloudBillboardFP_main bb color
compile f point_cloud_billboard_fp.cg pointCloudBillboardFP_selection bb selection

compile f point_cloud_billboard_fp.cg pointCloudBillboardSphereFP_main bb_sphere color
compile f point_cloud_billboard_fp.cg pointCloudBillboardSphereFP_depth bb_sphere depth
compile f point_cloud_billboard_fp.cg pointCloudBillboardSphereFP_Selection_main bb_sphere selection

compile f point_cloud_sprite_fp.cg pointCloudSpriteFP_main bb_sprite color

compile f point_cloud_box_fp.cg pointCloudBoxFP_main box color
compile f point_cloud_box_fp.cg pointCloudBoxFP_Selection_main box selection

compile f depth.cg depthFP_main passthrough depth
compile f passthrough.cg passthroughFP_main passthrough color

# vertex shaders

#compile v point_cloud_billboard_vp.cg pointCloudBillboardVP_main bb color
#compile v point_cloud_billboard_vp.cg pointCloudBillboardVP_Selection_main bb selection

#compile v point_cloud_point_vp.cg pointCloudPointVP_main point color
#compile v point_cloud_point_vp.cg pointCloudPointVP_Selection_main point selection

