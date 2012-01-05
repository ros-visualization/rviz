include_directories(.)

qt4_wrap_cpp(MOC_FILES
  camera_display.h
  image_display.h
  interactive_markers/integer_action.h
  interactive_markers/interactive_marker.h
)

set(SOURCE_FILES
  init.cpp
  axes_display.cpp
  camera_display.cpp
  grid_cells_display.cpp
  grid_display.cpp
  image_display.cpp
  interactive_marker_display.cpp
  interactive_markers/integer_action.cpp
  interactive_markers/interactive_marker_client.cpp
  interactive_markers/interactive_marker_control.cpp
  interactive_markers/interactive_marker.cpp
  laser_scan_display.cpp
  map_display.cpp
  marker_array_display.cpp
  marker_display.cpp
  markers/arrow_marker.cpp
  markers/line_list_marker.cpp
  markers/line_strip_marker.cpp
  markers/marker_base.cpp
  markers/marker_selection_handler.cpp
  markers/mesh_resource_marker.cpp
  markers/points_marker.cpp
  markers/shape_marker.cpp
  markers/text_view_facing_marker.cpp
  markers/triangle_list_marker.cpp
  odometry_display.cpp
  path_display.cpp
  point_cloud2_display.cpp
  point_cloud_base.cpp
  point_cloud_display.cpp
  point_cloud_transformers.cpp
  polygon_display.cpp
  pose_array_display.cpp
  pose_display.cpp
  range_display.cpp
  robot_model_display.cpp
  tf_display.cpp
  ${MOC_FILES}
)

rosbuild_add_library(default_plugin ${SOURCE_FILES})
                                    
target_link_libraries(default_plugin ${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})
rosbuild_link_boost(default_plugin thread signals filesystem system program_options)
