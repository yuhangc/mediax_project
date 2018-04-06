cmake_minimum_required(VERSION 3.0)
project(mediax_project)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs

  message_generation
  cmake_modules
  cv_bridge
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mediax_project
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## specify qt packages and directories
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDES})

## specify the aruco package and directories
find_package(aruco REQUIRED)
include_directories(${aruco_INCLUDE_DIRS})

## specify openCV package and directories
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## specify Eigen3 package and directories
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

## specify the json package and directories
find_package(nlohmann_json CONFIG REQUIRED)
include_directories(${JSON_INCLUDE_DIR})

## specify the json package and directories
find_package(OpenGL REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include/mediax_project
    external/Arduino
    ${catkin_INCLUDE_DIRS}
)

link_directories(${aruco_LIB_DIR})

# set source, header for the aruco vision tracking node
set(INSIDE_OUT_TRACKER_SOURCES
    src/inside_out_tracker.cpp
)

set(INSIDE_OUT_TRACKER_HEADERS
    include/mediax_project/inside_out_tracker.h
)

# set source, header for the aruco vision tracking node
set(TRACKER_NODE_SOURCES
    src/nodes/tracker_node.cpp
)

# set source, header for the aruco vision tracking node
set(TRACKER_NODELET_SOURCES
    src/nodelets/tracker_nodelet.cpp
)

# set source, header for the tracker calibration
set(TRACKER_CALIBRATION_SOURCES
    src/nodes/tracker_calibration.cpp
)

# set source, header for the serial manager
set(SERIAL_MANAGER_SOURCES
    src/serial_manager.cpp
    external/Arduino/CArduinoDevice.cpp
)

set(SERIAL_MANAGER_HEADERS
        include/mediax_project/serial_manager.h
        external/Arduino/CArduinoDevice.h
        )

# set source, header for the haptics driver
set(HAPTICS_SOURCES
    src/haptic_usb.cpp
    src/nodes/haptic_node.cpp
)

set(HAPTICS_HEADERS
    include/mediax_project/haptic_usb.h
)

# set source, header and the ui for the experiment main control window
set(EXP_CONTROL_SOURCES
    src/exp_main_window.cpp
    src/gl_tracking_display.cpp
    src/exp_control.cpp
)

set(EXP_CONTROL_HEADERS
    include/mediax_project/exp_main_window.h
    include/mediax_project/gl_tracking_display.h
)

set(EXP_CONTROL_FORMS
    ui/exp_main_window.ui
)

## Declare C++ libraries
add_library(serial_manager ${SERIAL_MANAGER_SOURCES} ${SERIAL_MANAGER_HEADERS})
target_link_libraries(serial_manager ${catkin_LIBRARIES})

add_library(inside_out_tracker ${INSIDE_OUT_TRACKER_SOURCES} ${INSIDE_OUT_TRACKER_HEADERS})
target_link_libraries(inside_out_tracker ${catkin_LIBRARIES} ${aruco_LIBS} ${OpenCV_LIBRARIES})

add_library(tracker_nodelet ${TRACKER_NODELET_SOURCES})
target_link_libraries(tracker_nodelet ${catkin_LIBRARIES} inside_out_tracker)

## Declare C++ executables
add_executable(tracker_node ${TRACKER_NODE_SOURCES})
target_link_libraries(tracker_node ${catkin_LIBRARIES} inside_out_tracker)

add_executable(tracker_calibration ${TRACKER_CALIBRATION_SOURCES})
target_link_libraries(tracker_calibration inside_out_tracker ${catkin_LIBRARIES})

add_executable(imu_manager_node src/nodes/imu_manager_node.cpp)
target_link_libraries(imu_manager_node ${catkin_LIBRARIES} serial_manager)

add_executable(opt_flow_bridge src/nodes/opt_flow_bridge.cpp)
target_link_libraries(opt_flow_bridge ${catkin_LIBRARIES} serial_manager)

add_executable(haptics_usb_node ${HAPTICS_SOURCES} ${HAPTICS_HEADERS})
target_link_libraries(haptics_usb_node ${catkin_LIBRARIES} usb-1.0 hidapi-libusb mccusb)

# compile the experiment main window application
qt5_wrap_cpp(MOC_SOURCES ${EXP_CONTROL_HEADERS})
qt5_wrap_ui(FORMS_HEADERS ${EXP_CONTROL_FORMS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(hri_exp_control ${EXP_CONTROL_SOURCES} ${EXP_CONTROL_HEADERS} ${MOC_SOURCES} ${FORMS_HEADERS})
target_link_libraries(hri_exp_control ${catkin_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
qt5_use_modules(hri_exp_control Widgets)
set_target_properties(hri_exp_control PROPERTIES OUTPUT_NAME exp_control)
