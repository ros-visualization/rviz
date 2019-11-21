set(EXPORT_HEADER_DIR "${CATKIN_DEVEL_PREFIX}/include")
include_directories("${CATKIN_DEVEL_PREFIX}/include")

add_subdirectory(rviz)
add_subdirectory(image_view)
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
# Python bindings are only support with Qt5, PyQt5, and PySide2 for Kinetic+.
if (UseQt5)
  add_subdirectory(python_bindings)
endif()
