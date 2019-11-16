add_subdirectory(rviz)
add_subdirectory(image_view)
if (CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
# Python bindings are only support with Qt5, PyQt5, and PySide2 for Kinetic+.
if (UseQt5)
  add_subdirectory(python_bindings)
endif()

# to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_EXCLUDES "*/test/*")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif()
