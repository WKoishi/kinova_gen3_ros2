cmake_minimum_required(VERSION 3.8)
project(kortex_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Explicitely specify the build type in case the user did not do it
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(message_package REQUIRED)

file(GLOB_RECURSE generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/generated/robot/*.cpp" "src/generated/simulation/*.cpp")
file(GLOB_RECURSE non_generated_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/driver/*.cpp")
file(GLOB_RECURSE test_files RELATIVE ${PROJECT_SOURCE_DIR} "src/non-generated/tests/*.cc")

# Find all auto-generated subdirectories in msg/generated
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/msg/generated ${PROJECT_SOURCE_DIR}/msg/generated/*)
set(msg_generated_dir_list "")
foreach(child ${children})
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/msg/generated/${child})
      list(APPEND msg_generated_dir_list ${child})
    endif()
endforeach()

# Find all auto-generated subdirectories in srv/generated
file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/srv/generated ${PROJECT_SOURCE_DIR}/srv/generated/*)
set(srv_generated_dir_list "")
foreach(child ${children})
    if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/srv/generated/${child})
      list(APPEND srv_generated_dir_list ${child})
    endif()
endforeach()

set(msg_file_list "")
foreach(folder ${msg_generated_dir_list})
  file(GLOB msg_file_abs "${PROJECT_SOURCE_DIR}/msg/generated/${folder}/*.msg")
  set(msg_file_rel "")
  foreach(temp ${msg_file_abs})
    string(REPLACE "${PROJECT_SOURCE_DIR}/" "" temp ${temp})
    list(APPEND msg_file_rel ${temp})
  endforeach()
  list(APPEND msg_file_list ${msg_file_rel})
endforeach()

list(APPEND msg_file_list "msg/non_generated/ApiOptions.msg")
list(APPEND msg_file_list "msg/non_generated/KortexError.msg")

set(srv_file_list "")
foreach(folder ${srv_generated_dir_list})
  file(GLOB srv_file_abs "${PROJECT_SOURCE_DIR}/srv/generated/${folder}/*.srv")
  set(srv_file_rel "")
  foreach(temp ${srv_file_abs})
    string(REPLACE "${PROJECT_SOURCE_DIR}/" "" temp ${temp})
    list(APPEND srv_file_rel ${temp})
  endforeach()
  list(APPEND srv_file_list ${srv_file_rel})
endforeach()

list(APPEND srv_file_list "srv/non_generated/SetApiOptions.srv")
list(APPEND srv_file_list "srv/non_generated/SetDeviceID.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_file_list}
  ${srv_file_list}
  DEPENDENCIES std_msgs
)

# message(WARNING "${msg_file_rel}")
# message(WARNING "--------")
# message(WARNING "${srv_file_list}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
