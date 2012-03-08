add_subdirectory(default_plugin)

include_directories(.)

# Build the SVN version number or (if not in an SVN directory) ROS
# stack version number into the source so it can be printed at
# run-time.
set(VERSION_FILE ${PROJECT_SOURCE_DIR}/build/version.cpp)
execute_process(COMMAND          svnversion -n ${PROJECT_SOURCE_DIR}
                OUTPUT_VARIABLE  SVN_VERSION)
if(${SVN_VERSION} MATCHES "^exported$")
  execute_process(COMMAND          rosversion visualization
                  OUTPUT_VARIABLE  SVN_VERSION
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
configure_file(version.cpp.in ${VERSION_FILE} @ONLY)

# We create one lib with the C++...

qt4_wrap_cpp(MOC_FILES
  display.h
  displays_panel.h
  display_wrapper.h
  help_panel.h
  new_object_dialog.h
  panel.h
  panel_dock_widget.h
  properties/color_editor.h
  properties/edit_enum_item.h
  properties/enum_item.h
  properties/line_edit_with_button.h
  properties/property_manager.h
  properties/property_tree_widget.h
  properties/property_tree_with_help.h
  properties/ros_topic_dialog.h
  properties/ros_topic_editor.h
  properties/ros_topic_tree.h
  render_panel.h
  scaled_image_widget.h
  screenshot_dialog.h
  selection_panel.h
  selection/selection_manager.h
  time_panel.h
  tool_properties_panel.h
  view_controller.h
  views_panel.h
  visualization_frame.h
  visualization_manager.h
  visualization_panel.h
  visualizer_app.h
  wait_for_master_dialog.h
  widget_geometry_change_detector.h
  )

rosbuild_add_library(${PROJECT_NAME}
                                    help_panel.cpp
                                    loading_dialog.cpp
                                    visualization_frame.cpp
                                    wait_for_master_dialog.cpp
                                    visualizer_app.cpp
                                    ogre_helpers/initialization.cpp
                                    ogre_helpers/render_system.cpp
                                    ogre_helpers/render_widget.cpp
                                    ogre_helpers/point_cloud.cpp
                                    ogre_helpers/object.cpp
                                    ogre_helpers/shape.cpp
                                    ogre_helpers/grid.cpp
                                    ogre_helpers/arrow.cpp
                                    ogre_helpers/axes.cpp
                                    ogre_helpers/billboard_line.cpp
                                    ogre_helpers/camera_base.cpp
                                    ogre_helpers/movable_text.cpp
                                    ogre_helpers/qt_ogre_render_window.cpp
                                    ogre_helpers/orthographic.cpp
                                    panel.cpp
                                    properties/property_tree_with_help.cpp
                                    properties/property_widget_item.cpp
                                    properties/property_tree_widget.cpp
                                    properties/property.cpp
                                    properties/property_manager.cpp
                                    properties/ros_topic_dialog.cpp
                                    properties/ros_topic_tree.cpp
                                    properties/ros_topic_editor.cpp
                                    properties/color_editor.cpp
                                    properties/color_item.cpp
                                    properties/enum_item.cpp
                                    properties/edit_enum_item.cpp
                                    properties/compound_widget_item.cpp
                                    properties/line_edit_with_button.cpp
				    scaled_image_widget.cpp
				    screenshot_dialog.cpp
                                    config.cpp
                                    image/ros_image_texture.cpp
                                    tools/tool.cpp
                                    tools/move_tool.cpp
                                    tools/pose_tool.cpp
                                    tools/goal_tool.cpp
                                    tools/initial_pose_tool.cpp
                                    tools/selection_tool.cpp
                                    tools/interaction_tool.cpp
                                    mesh_loader.cpp
                                    render_panel.cpp
                                    displays_panel.cpp
                                    views_panel.cpp
                                    time_panel.cpp
                                    selection_panel.cpp
                                    tool_properties_panel.cpp
                                    frame_manager.cpp
                                    visualization_manager.cpp
                                    visualization_panel.cpp
                                    display.cpp
                                    display_wrapper.cpp
                                    view_controller.cpp
                                    view_controllers/xy_orbit_view_controller.cpp
                                    view_controllers/orbit_view_controller.cpp
                                    view_controllers/fps_view_controller.cpp
                                    view_controllers/fixed_orientation_ortho_view_controller.cpp
                                    new_object_dialog.cpp
				    panel_dock_widget.cpp
                                    robot/tf_link_updater.cpp
                                    robot/robot_link.cpp
                                    robot/robot.cpp
                                    selection/selection_manager.cpp
                                    selection/selection_handler.cpp
                                    uniform_string_stream.cpp
                                    widget_geometry_change_detector.cpp
                                    ${VERSION_FILE}
                                    ${MOC_FILES}
                                    )

##### If gtk turns out to be the best way to get the window position, use this to compile it.
#
# target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${GTK2_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} assimp yaml-cpp ${OPENGL_LIBRARIES})
rosbuild_link_boost(${PROJECT_NAME} thread signals filesystem system program_options)

if(APPLE)
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
endif(APPLE)

########### The rviz executable ###########
rosbuild_add_executable(executable main.cpp)
target_link_libraries(executable ${PROJECT_NAME} ${QT_LIBRARIES})
set_target_properties(executable
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
                      PREFIX "")
