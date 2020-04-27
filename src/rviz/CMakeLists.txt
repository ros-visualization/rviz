find_package(yaml-cpp REQUIRED)

include_directories(${YAML_CPP_INCLUDE_DIR})

if(UNIX AND NOT APPLE)
  find_package(X11 REQUIRED)
endif()

add_subdirectory(default_plugin)

# Build the version number and other build-time constants into the
# source for access at run-time.
set(ENV_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/env_config.cpp)
set(RVIZ_VERSION ${rviz_VERSION})
message(STATUS "making version ${RVIZ_VERSION}.")
#catkin_lint: ignore_once env_var
set(ROS_DISTRO $ENV{ROS_DISTRO})

configure_file(env_config.cpp.in ${ENV_CONFIG_FILE} @ONLY)

# We create one lib with the C++...

add_library(${PROJECT_NAME}
  bit_allocator.cpp
  config.cpp
  display.cpp
  display.cpp
  display_context.h
  display_factory.cpp
  display_group.cpp
  displays_panel.cpp
  failed_display.cpp
  failed_panel.cpp
  failed_tool.cpp
  failed_view_controller.cpp
  frame_manager.cpp
  load_resource.cpp
  frame_position_tracking_view_controller.cpp
  geometry.cpp
  help_panel.cpp
  image/ros_image_texture.cpp
  image/image_display_base.cpp
  loading_dialog.cpp
  message_filter_display.h
  mesh_loader.cpp
  new_object_dialog.cpp
  add_display_dialog.cpp
  ogre_helpers/apply_visibility_bits.cpp
  ogre_helpers/arrow.cpp
  ogre_helpers/axes.cpp
  ogre_helpers/billboard_line.cpp
  ogre_helpers/camera_base.cpp
  ogre_helpers/grid.cpp
  ogre_helpers/initialization.cpp
  ogre_helpers/line.cpp
  ogre_helpers/movable_text.cpp
  ogre_helpers/object.cpp
  ogre_helpers/ogre_logging.cpp
  ogre_helpers/ogre_render_queue_clearer.cpp
  ogre_helpers/orthographic.cpp
  ogre_helpers/point_cloud.cpp
  ogre_helpers/qt_ogre_render_window.cpp
  ogre_helpers/render_system.cpp
  ogre_helpers/render_widget.cpp
  ogre_helpers/shape.cpp
  ogre_helpers/mesh_shape.cpp
  panel.cpp
  panel_dock_widget.cpp
  panel_factory.cpp
  preferences_dialog.cpp
  properties/bool_property.cpp
  properties/color_editor.cpp
  properties/color_property.cpp
  properties/combo_box.cpp
  properties/display_visibility_property.cpp
  properties/display_group_visibility_property.cpp
  properties/editable_combo_box.cpp
  properties/editable_enum_property.cpp
  properties/enum_property.cpp
  properties/float_edit.cpp
  properties/float_property.cpp
  properties/int_property.cpp
  properties/line_edit_with_button.cpp
  properties/parse_color.cpp
  properties/property.cpp
  properties/property_tree_delegate.cpp
  properties/property_tree_model.cpp
  properties/property_tree_widget.cpp
  properties/property_tree_with_help.cpp
  properties/quaternion_property.cpp
  properties/ros_topic_property.cpp
  properties/splitter_handle.cpp
  properties/status_list.cpp
  properties/status_property.cpp
  properties/string_property.cpp
  properties/tf_frame_property.cpp
  properties/vector_property.cpp
  render_panel.cpp
  robot/robot_link.cpp
  robot/robot_joint.cpp
  robot/robot.cpp
  robot/tf_link_updater.cpp
  scaled_image_widget.cpp
  screenshot_dialog.cpp
  selection_panel.cpp
  selection/selection_handler.cpp
  selection/selection_manager.cpp
  splash_screen.cpp
  time_panel.cpp
  tool.cpp
  tool_manager.cpp
  uniform_string_stream.cpp
  view_controller.cpp
  view_manager.cpp
  views_panel.cpp
  visualization_frame.cpp
  visualization_manager.cpp
  visualizer_app.cpp
  wait_for_master_dialog.cpp
  widget_geometry_change_detector.cpp
  tool_properties_panel.cpp
  yaml_config_reader.cpp
  yaml_config_writer.cpp

  ${ENV_CONFIG_FILE}
)

include(GenerateExportHeader)
set(EXPORT_FILE_NAME ${EXPORT_HEADER_DIR}/rviz/${PROJECT_NAME}_export.h)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${EXPORT_FILE_NAME})
if(NOT WIN32)
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${OGRE_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${rviz_ADDITIONAL_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  ${X11_X11_LIB}
  ${ASSIMP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

########### The rviz executable ###########
add_executable(executable main.cpp)

if(NOT WIN32)
  set_target_properties(executable PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()

target_link_libraries(executable ${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_LIBRARIES})

set_target_properties(executable
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

install(TARGETS executable
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

#catkin_lint: ignore_once external_file
install(FILES
  ${EXPORT_FILE_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS executable
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
