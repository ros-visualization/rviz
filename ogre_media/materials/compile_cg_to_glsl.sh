#!/bin/bash

set -e

#Compiles all cg scripts into glsl 

compile () { 
  OUT_DIR="glsl/$4"
  OUT_FILE="$OUT_DIR/$5_$1p.glsl"
  
  echo "-------------------------------------------------"
  echo "type   = $1"
  echo "in     = $2"
  echo "main() = $3"
  echo "out    = $4/$5"
  echo ""
  
  if [ ! -d $OUT_DIR ] 
    then  
  	mkdir $OUT_DIR
  fi
  
  # compile
  cgc cg/$2 -entry $3 -o $OUT_FILE -profile glsl$1
  
  # replace messed-up uniform var names
  
  sed -i 's/_highlight1/highlight/g' $OUT_FILE
  
  sed -i 's/_alpha1/alpha/g' $OUT_FILE
  sed -i 's/_size2/size/g' $OUT_FILE
  sed -i 's/_worldviewproj_matrix/worldviewproj_matrix/g' $OUT_FILE
  sed -i 's/_worldview_matrix/worldview_matrix/g' $OUT_FILE
  sed -i 's/_projection_matrix/projection_matrix/g' $OUT_FILE
  sed -i 's/_viewport_height_pixels/viewport_height_pixels/g' $OUT_FILE
  sed -i 's/_worldviewproj2/worldviewproj/g' $OUT_FILE
  sed -i 's/_camera_pos2/camera_pos/g' $OUT_FILE
  
  # correct for vec4[4] instead of mat4
  sed -i 's/vec4 \(.*\)\[4\]/mat4 \1/g' $OUT_FILE
  
}


# (shaders that are commented out are already re-written)

# fragment shaders

#compile f point_cloud_billboard_fp.cg pointCloudBillboardFP_main bb color
compile f point_cloud_billboard_fp.cg pointCloudBillboardFP_selection bb selection

compile f point_cloud_billboard_fp.cg pointCloudBillboardSphereFP_main bb_sphere color
compile f point_cloud_billboard_fp.cg pointCloudBillboardSphereFP_depth bb_sphere depth
compile f point_cloud_billboard_fp.cg pointCloudBillboardSphereFP_Selection_main bb_sphere selection

#compile f point_cloud_sprite_fp.cg pointCloudSpriteFP_main bb_sprite color

compile f point_cloud_box_fp.cg pointCloudBoxFP_main box color
compile f point_cloud_box_fp.cg pointCloudBoxFP_Selection_main box selection

compile f depth.cg depthFP_main passthrough depth
compile f passthrough.cg passthroughFP_main passthrough color

# vertex shaders

# Note: vertex shader output of cgc does not work
# because e.g. is float4x4 is mislabeled as vec4[4] instead of mat4

#compile v point_cloud_sprite_vp.cg pointCloudSpriteVP_main bb_sprite color

#compile v point_cloud_billboard_vp.cg pointCloudBillboardVP_main bb color

#compile v point_cloud_billboard_vp.cg pointCloudBillboardVP_main bb color
#compile v point_cloud_billboard_vp.cg pointCloudBillboardVP_Selection_main bb selection

#compile v point_cloud_point_vp.cg pointCloudPointVP_main point color
#compile v point_cloud_point_vp.cg pointCloudPointVP_Selection_main point selection

