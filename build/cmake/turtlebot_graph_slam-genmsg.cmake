# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "turtlebot_graph_slam: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iturtlebot_graph_slam:/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(turtlebot_graph_slam_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" NAME_WE)
add_custom_target(_turtlebot_graph_slam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "turtlebot_graph_slam" "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" "nav_msgs/Odometry:geometry_msgs/Point:std_msgs/MultiArrayDimension:geometry_msgs/Twist:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/TransformStamped:std_msgs/MultiArrayLayout:std_msgs/Float64MultiArray:geometry_msgs/TwistWithCovariance:geometry_msgs/Vector3:geometry_msgs/PoseWithCovariance:geometry_msgs/Transform:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" NAME_WE)
add_custom_target(_turtlebot_graph_slam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "turtlebot_graph_slam" "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" "geometry_msgs/Point:std_msgs/MultiArrayDimension:geometry_msgs/Pose:std_msgs/Header:std_msgs/MultiArrayLayout:std_msgs/Float64MultiArray:geometry_msgs/PoseArray:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" NAME_WE)
add_custom_target(_turtlebot_graph_slam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "turtlebot_graph_slam" "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/turtlebot_graph_slam
)
_generate_msg_cpp(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Services
_generate_srv_cpp(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Module File
_generate_module_cpp(turtlebot_graph_slam
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/turtlebot_graph_slam
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(turtlebot_graph_slam_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(turtlebot_graph_slam_generate_messages turtlebot_graph_slam_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_cpp _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_cpp _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_cpp _turtlebot_graph_slam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(turtlebot_graph_slam_gencpp)
add_dependencies(turtlebot_graph_slam_gencpp turtlebot_graph_slam_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS turtlebot_graph_slam_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/turtlebot_graph_slam
)
_generate_msg_eus(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Services
_generate_srv_eus(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Module File
_generate_module_eus(turtlebot_graph_slam
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/turtlebot_graph_slam
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(turtlebot_graph_slam_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(turtlebot_graph_slam_generate_messages turtlebot_graph_slam_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_eus _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_eus _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_eus _turtlebot_graph_slam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(turtlebot_graph_slam_geneus)
add_dependencies(turtlebot_graph_slam_geneus turtlebot_graph_slam_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS turtlebot_graph_slam_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/turtlebot_graph_slam
)
_generate_msg_lisp(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Services
_generate_srv_lisp(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Module File
_generate_module_lisp(turtlebot_graph_slam
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/turtlebot_graph_slam
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(turtlebot_graph_slam_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(turtlebot_graph_slam_generate_messages turtlebot_graph_slam_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_lisp _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_lisp _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_lisp _turtlebot_graph_slam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(turtlebot_graph_slam_genlisp)
add_dependencies(turtlebot_graph_slam_genlisp turtlebot_graph_slam_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS turtlebot_graph_slam_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/turtlebot_graph_slam
)
_generate_msg_nodejs(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Services
_generate_srv_nodejs(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Module File
_generate_module_nodejs(turtlebot_graph_slam
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/turtlebot_graph_slam
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(turtlebot_graph_slam_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(turtlebot_graph_slam_generate_messages turtlebot_graph_slam_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_nodejs _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_nodejs _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_nodejs _turtlebot_graph_slam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(turtlebot_graph_slam_gennodejs)
add_dependencies(turtlebot_graph_slam_gennodejs turtlebot_graph_slam_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS turtlebot_graph_slam_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam
)
_generate_msg_py(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Services
_generate_srv_py(turtlebot_graph_slam
  "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam
)

### Generating Module File
_generate_module_py(turtlebot_graph_slam
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(turtlebot_graph_slam_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(turtlebot_graph_slam_generate_messages turtlebot_graph_slam_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/tfArray.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_py _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/msg/keyframe.msg" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_py _turtlebot_graph_slam_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/srv/ResetFilter.srv" NAME_WE)
add_dependencies(turtlebot_graph_slam_generate_messages_py _turtlebot_graph_slam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(turtlebot_graph_slam_genpy)
add_dependencies(turtlebot_graph_slam_genpy turtlebot_graph_slam_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS turtlebot_graph_slam_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/turtlebot_graph_slam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/turtlebot_graph_slam
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(turtlebot_graph_slam_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(turtlebot_graph_slam_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(turtlebot_graph_slam_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/turtlebot_graph_slam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/turtlebot_graph_slam
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(turtlebot_graph_slam_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(turtlebot_graph_slam_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(turtlebot_graph_slam_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/turtlebot_graph_slam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/turtlebot_graph_slam
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(turtlebot_graph_slam_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(turtlebot_graph_slam_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(turtlebot_graph_slam_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/turtlebot_graph_slam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/turtlebot_graph_slam
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(turtlebot_graph_slam_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(turtlebot_graph_slam_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(turtlebot_graph_slam_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/turtlebot_graph_slam
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(turtlebot_graph_slam_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(turtlebot_graph_slam_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(turtlebot_graph_slam_generate_messages_py nav_msgs_generate_messages_py)
endif()
