cmake_minimum_required(VERSION 3.8)
project(kortex_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CONAN_TARGET_PLATFORM "x86")

# Explicitely specify the build type in case the user did not do it
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
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

set(action_file_list "action/non_generated/FollowCartesianTrajectory.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_file_list}
  ${srv_file_list}
  ${action_file_list}
  DEPENDENCIES std_msgs builtin_interfaces unique_identifier_msgs action_msgs
)

# Include conan.cmake module and download Kortex API from artifactory
include(${PROJECT_SOURCE_DIR}/cmake/conan.cmake)
conan_check(REQUIRED)
conan_add_remote(NAME kinova_public
                URL https://artifactory.kinovaapps.com/artifactory/api/conan/conan-public)
if("${CONAN_TARGET_PLATFORM}" STREQUAL "x86")
  conan_cmake_run(CONANFILE conanfile.py
                  UPDATE
                  BASIC_SETUP CMAKE_TARGETS
                  NO_OUTPUT_DIRS
                  SETTINGS kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=5 kortex_api_cpp:compiler.libcxx=libstdc++11)
elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "artik710")
  conan_cmake_run(CONANFILE conanfile.py
                  UPDATE
                  BASIC_SETUP CMAKE_TARGETS
                  NO_OUTPUT_DIRS
                  SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=5 kortex_api_cpp:compiler.libcxx=libstdc++11
                  ENV TARGET=artik710)
elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "imx6")
  conan_cmake_run(CONANFILE conanfile.py
                  UPDATE
                  BASIC_SETUP CMAKE_TARGETS
                  NO_OUTPUT_DIRS
                  SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=6.4 kortex_api_cpp:compiler.libcxx=libstdc++11
                  ENV TARGET=imx6)
elseif("${CONAN_TARGET_PLATFORM}" STREQUAL "jetson")
  conan_cmake_run(CONANFILE conanfile.py
                  UPDATE
                  BASIC_SETUP CMAKE_TARGETS
                  NO_OUTPUT_DIRS
                  SETTINGS kortex_api_cpp:arch=armv7 kortex_api_cpp:compiler=gcc kortex_api_cpp:compiler.version=7 kortex_api_cpp:compiler.libcxx=libstdc++11
                  ENV TARGET=jetson)
endif()

include_directories(include include/${PROJECT_NAME})
include_directories(include ${Boost_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/include)

include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include)
link_directories(${PROJECT_SOURCE_DIR}/../kortex_api/lib/release)

# Generated files library
add_library(kortex_driver_generated_files ${generated_files})

# link to ros generated library
ament_target_dependencies(kortex_driver_generated_files
  rclcpp
  std_msgs
  builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kortex_driver_generated_files "${cpp_typesupport_target}")

target_link_libraries(kortex_driver_generated_files CONAN_PKG::kortex_api_cpp ${catkin_LIBRARIES} gcov)

# Remove deprecation warnings from Kortex API
target_compile_options(kortex_driver_generated_files PRIVATE "-Wno-deprecated-declarations")

# Non-generated files library (driver implementation)
add_library(kortex_arm_driver_implementation ${non_generated_files})
target_link_libraries(kortex_arm_driver_implementation CONAN_PKG::kortex_api_cpp ${catkin_LIBRARIES} gcov kortex_driver_generated_files)
add_dependencies(kortex_arm_driver_implementation kortex_driver_generated_files)

# Entry point executable for the driver
add_executable(kortex_arm_driver src/non-generated/main.cpp)
target_link_libraries(kortex_arm_driver CONAN_PKG::kortex_api_cpp ${catkin_LIBRARIES} gcov kortex_arm_driver_implementation)
add_dependencies(kortex_arm_driver kortex_arm_driver_implementation)

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
