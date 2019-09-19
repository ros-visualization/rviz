add_executable(rviz_image_view
  image_view.cpp
  main.cpp
)
if(NOT WIN32)
  set_target_properties(rviz_image_view PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()

target_link_libraries(rviz_image_view
  ${OGRE_OV_LIBRARIES_ABS}
  ${rviz_ADDITIONAL_LIBRARIES}
  ${PROJECT_NAME}
)

if(APPLE)
    target_link_libraries(rviz_image_view "-framework OpenGL" "-framework Cocoa")
endif()

set_target_properties(rviz_image_view
                      PROPERTIES OUTPUT_NAME image_view
                      PREFIX "")
