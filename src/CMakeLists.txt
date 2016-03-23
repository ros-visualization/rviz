add_subdirectory(rviz)
add_subdirectory(image_view)
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
# Disable Python bindings when using Qt5.
# Will be re-enabled in Kinetic where our Qt Python packages will support Qt5.
if (NOT UseQt5)
  add_subdirectory(python_bindings)
endif()
