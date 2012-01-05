# Build file for Python bindings via shiboken and pyside.

find_package(GeneratorRunner REQUIRED)
find_package(Shiboken REQUIRED)
find_package(PySide REQUIRED)
find_package(PythonLibs REQUIRED)

# List of classes with python bindings in wx version of rviz preserved
# here for inspiration.
# 
# set_source_files_properties(${SWIG_OUTPUT_CPP_FILE} PROPERTIES GENERATED true)
# add_custom_command(OUTPUT   ${SWIG_OUTPUT_CPP_FILE} 
#                             ${SWIG_OUTPUT_PYTHON_FILE}
#                    COMMAND  ${SWIG_COMMAND}
#                    DEPENDS  ${SWIG_INTERFACE_FILE}
#                             display.i
#                             visualization_manager.i
#                             visualization_panel.i
#                             visualization_frame.i
#                             helpers/color.i
#                             visualization_manager.h
#                             visualization_panel.h
#                             visualization_frame.h    
#                             generated/rviz_generated.h
#                             display.h
#                             helpers/color.h
#                             status_level.h)

# To add a new rviz class to the bindings, add it:
# - in rviz_shiboken_SRCS below, like rviz_myclass_wrapper.cpp
# - in rviz_HDRS below, like ../rviz/my_class.h
# - in global.h, like #include <rviz/my_class.h>
# - in typesystem.xml, like <object-type name="rviz::MyClass"/>

set(rviz_shiboken_SRCS
  librviz_shiboken/librviz_shiboken_module_wrapper.cpp
  librviz_shiboken/rviz_visualizationframe_wrapper.cpp
  librviz_shiboken/rviz_visualizationpanel_wrapper.cpp
  librviz_shiboken/rviz_wrapper.cpp
)

set(rviz_HDRS
  ../rviz/visualization_frame.h
  ../rviz/visualization_panel.h
)

set(rviz_shiboken_INCLUDE_DIRECTORIES
  ${PYTHON_INCLUDE_DIR}
  ${SHIBOKEN_INCLUDE_DIR}
  ${PYSIDE_INCLUDE_DIR}
  ${PYSIDE_INCLUDE_DIR}/QtCore
  ${PYSIDE_INCLUDE_DIR}/QtGui
  ../rviz
)

set(rviz_shiboken_LINK_LIBRARIES
  ${QT_QTCORE_LIBRARY}
  ${QT_QTGUI_LIBRARY}
  ${SHIBOKEN_PYTHON_LIBRARIES}
  ${SHIBOKEN_LIBRARY}
  ${PYSIDE_LIBRARY}
  ${PROJECT_NAME}
)

# See ticket https://code.ros.org/trac/ros-pkg/ticket/5219
set(QT_INCLUDE_DIR_WITH_COLONS "")
foreach(dir ${QT_INCLUDE_DIR})
  set(QT_INCLUDE_DIR_WITH_COLONS "${QT_INCLUDE_DIR_WITH_COLONS}:${dir}")
endforeach()

# Add to custom command below for verbose debugging: --debug-level=full
add_custom_command(
  OUTPUT ${rviz_shiboken_SRCS}
  COMMAND ${GENERATORRUNNER_BINARY}
    --generatorSet=shiboken
    --include-paths=..:../rviz:${QT_INCLUDE_DIR_WITH_COLONS}
    --typesystem-paths=${PYSIDE_TYPESYSTEMS}
    --output-directory=${PROJECT_SOURCE_DIR}/build/src/python_bindings
    global.h
    typesystem.xml
  DEPENDS global.h typesystem.xml ${rviz_HDRS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/python_bindings
  COMMENT "Running Shiboken generator for librviz Python bindings..."
)

include_directories(rviz_shiboken ${rviz_shiboken_INCLUDE_DIRECTORIES})
rosbuild_add_library(rviz_shiboken ${rviz_shiboken_SRCS})
target_link_libraries(rviz_shiboken ${rviz_shiboken_LINK_LIBRARIES})
