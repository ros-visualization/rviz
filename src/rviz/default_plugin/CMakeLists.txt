include_directories(.)

set(SOURCE_FILES
  axes_display.cpp
  camera_display.cpp
  covariance_visual.cpp
  covariance_property.cpp
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
  marker_utils.cpp
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
  point_cloud_transformer.h
  point_cloud_transformers.cpp
  polygon_display.cpp
  pose_array_display.cpp
  pose_display.cpp
  pose_with_covariance_display.cpp
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
  view_controllers/third_person_follower_view_controller.cpp
  view_controllers/fixed_orientation_ortho_view_controller.cpp
  view_controllers/fps_view_controller.cpp
  wrench_display.cpp
  wrench_visual.cpp
)

add_library(${rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME} ${SOURCE_FILES})
if(NOT WIN32)
  set_target_properties(${rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME}
    PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()
target_link_libraries(${rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME}
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
  ${QT_LIBRARIES}
)

install(TARGETS ${rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Generate to the devel space so the extras file can include it from the devel space.
file(GENERATE
  OUTPUT "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/default_plugin_location.cmake"
  CONTENT "set(rviz_DEFAULT_PLUGIN_FILE_NAME $<TARGET_FILE_NAME:${rviz_DEFAULT_PLUGIN_LIBRARY_TARGET_NAME}>)"
)
# Install from the devel space to the install space.
install(FILES "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/default_plugin_location.cmake"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)
