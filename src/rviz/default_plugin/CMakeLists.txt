include_directories(.)

qt4_wrap_cpp(MOC_FILES
  axes_display.h
  camera_display.h
  depth_cloud_display.h
  effort_display.h
  fluid_pressure_display.h
  grid_cells_display.h
  grid_display.h
  illuminance_display.h
  image_display.h
  interactive_marker_display.h
  interactive_markers/integer_action.h
  interactive_markers/interactive_marker.h
  laser_scan_display.h
  map_display.h
  marker_display.h
  marker_array_display.h
  odometry_display.h
  path_display.h
  point_display.h
  point_cloud_common.h
  point_cloud_display.h
  point_cloud_transformer.h
  point_cloud_transformers.h
  point_cloud2_display.h
  polygon_display.h
  pose_array_display.h
  pose_display.h
  range_display.h
  relative_humidity_display.h
  robot_model_display.h
  temperature_display.h
  tf_display.h
  tools/goal_tool.h
  tools/initial_pose_tool.h
  tools/interaction_tool.h
  tools/point_tool.h
  view_controllers/orbit_view_controller.h
  view_controllers/xy_orbit_view_controller.h
  view_controllers/fixed_orientation_ortho_view_controller.h
  view_controllers/fps_view_controller.h
  wrench_display.h
)

set(SOURCE_FILES
  axes_display.cpp
  camera_display.cpp
  depth_cloud_display.cpp
  depth_cloud_mld.cpp
  effort_display.cpp
  effort_visual.cpp
  fluid_pressure_display.cpp
  grid_cells_display.cpp
  grid_display.cpp
  illuminance_display.cpp
  image_display.cpp
  interactive_marker_display.cpp
  interactive_markers/integer_action.cpp
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
  point_display.cpp
  point_visual.cpp
  point_cloud2_display.cpp
  point_cloud_common.cpp
  point_cloud_display.cpp
  point_cloud_transformers.cpp
  polygon_display.cpp
  pose_array_display.cpp
  pose_display.cpp
  range_display.cpp
  relative_humidity_display.cpp
  robot_model_display.cpp
  temperature_display.cpp
  tf_display.cpp
  tools/focus_tool.cpp
  tools/measure_tool.cpp
  tools/move_tool.cpp
  tools/point_tool.cpp
  tools/pose_tool.cpp
  tools/goal_tool.cpp
  tools/initial_pose_tool.cpp
  tools/selection_tool.cpp
  tools/interaction_tool.cpp
  view_controllers/orbit_view_controller.cpp
  view_controllers/xy_orbit_view_controller.cpp
  view_controllers/fixed_orientation_ortho_view_controller.cpp
  view_controllers/fps_view_controller.cpp
  wrench_display.cpp
  wrench_visual.cpp
  ${MOC_FILES}
)

add_library(default_plugin ${SOURCE_FILES})
target_link_libraries(default_plugin ${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS} ${Boost_LIBRARIES})

install(TARGETS default_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#message("TODO: stop installing libdefault_plugin.so into share/rviz after pluginlib starts looking for it in lib.")
#install(TARGETS default_plugin
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
