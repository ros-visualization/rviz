add_subdirectory(default_plugin)

include_directories(.)

# Build the version number and other build-time constants into the
# source for access at run-time.
set(ENV_CONFIG_FILE ${CMAKE_CURRENT_BINARY_DIR}/env_config.cpp)
set(RVIZ_VERSION ${rviz_VERSION})
message("making version ${RVIZ_VERSION}.")
set(ROS_DISTRO $ENV{ROS_DISTRO})

configure_file(env_config.cpp.in ${ENV_CONFIG_FILE} @ONLY)

# We create one lib with the C++...

qt4_wrap_cpp(MOC_FILES
  display_context.h
  display_group.h
  display.h
  displays_panel.h
  failed_panel.h
  frame_manager.h
  frame_position_tracking_view_controller.h
  help_panel.h
  image/image_display_base.h
  loading_dialog.h
  message_filter_display.h
  new_object_dialog.h
  add_display_dialog.h
  panel_dock_widget.h
  panel.h
  properties/bool_property.h
  properties/color_editor.h
  properties/color_property.h
  properties/combo_box.h
  properties/display_visibility_property.h
  properties/display_group_visibility_property.h
  properties/editable_combo_box.h
  properties/editable_enum_property.h
  properties/enum_property.h
  properties/float_edit.h
  properties/float_property.h
  properties/int_property.h
  properties/line_edit_with_button.h
  properties/property.h
  properties/property_tree_delegate.h
  properties/property_tree_model.h
  properties/property_tree_widget.h
  properties/property_tree_with_help.h
  properties/quaternion_property.h
  properties/ros_topic_property.h
  properties/splitter_handle.h
  properties/status_list.h
  properties/status_property.h
  properties/string_property.h
  properties/tf_frame_property.h
  properties/vector_property.h
  render_panel.h
  robot/robot.h
  robot/robot_link.h
  robot/robot_joint.h
  scaled_image_widget.h
  screenshot_dialog.h
  selection/selection_manager.h
  splash_screen.h
  time_panel.h
  tool_manager.h
  tool.h
  view_controller.h
  view_manager.h
  views_panel.h
  visualization_frame.h
  visualization_manager.h
  visualizer_app.h
  wait_for_master_dialog.h
  widget_geometry_change_detector.h
  selection_panel.h
  tool_properties_panel.h
)

add_library( ${PROJECT_NAME}
  bit_allocator.cpp
  config.cpp
  display.cpp
  display.cpp
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
  ogre_helpers/stl_loader.cpp
  panel.cpp
  panel_dock_widget.cpp
  panel_factory.cpp
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
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OGRE_OV_LIBRARIES_ABS}
  ${OPENGL_LIBRARIES}
  ${QT_LIBRARIES}
  ${rviz_ADDITIONAL_LIBRARIES}
  assimp
  yaml-cpp
)

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150" )
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DRVIZ_HAVE_YAMLCPP_05)
endif(NEW_YAMLCPP_FOUND)

########### The rviz executable ###########
add_executable(executable main.cpp)

target_link_libraries(executable ${PROJECT_NAME} ${QT_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})
set_target_properties(executable
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

install(TARGETS executable ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(TARGETS executable
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
