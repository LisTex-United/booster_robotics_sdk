cmake_policy(SET CMP0048 NEW)
project(
  ${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

cmake_minimum_required(VERSION 3.15...3.27)

option(BUILD_PYTHON_BINDING "Build Python Binding" off)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)

# ── move_robot_ros_node ────────────────────────────────────────────────────────
# Defined BEFORE link_libraries() so it does NOT inherit the Booster SDK libs.
# This keeps its DDS stack 100% ROS-Humble — no Booster FastDDS conflict.
find_package(rclcpp QUIET)
find_package(trajectory_msgs QUIET)
if(rclcpp_FOUND AND trajectory_msgs_FOUND)
    add_executable(move_robot_ros_node example/high_level/move_robot_ros_node.cpp)
    target_include_directories(move_robot_ros_node PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${trajectory_msgs_INCLUDE_DIRS}
    )
    target_link_libraries(move_robot_ros_node PRIVATE
        rclcpp::rclcpp
        trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
    )
    message(STATUS "move_robot_ros_node: enabled")
else()
    message(STATUS "move_robot_ros_node: skipped (rclcpp/trajectory_msgs not found)")
endif()

# ── move_robot_bridge ─────────────────────────────────────────────────────────
# Also defined BEFORE link_libraries() and uses explicit /usr/local/lib paths,
# so it always links against FastDDS v2.13 regardless of whether the ROS
# environment is sourced at build time.
add_executable(move_robot_bridge example/high_level/move_robot_bridge.cpp)
target_include_directories(move_robot_bridge BEFORE PRIVATE
    ${PROJECT_SOURCE_DIR}/third_party/include
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(move_robot_bridge PRIVATE
    /usr/local/lib/libbooster_robotics_sdk.a
    /usr/local/lib/libfastrtps.so
    /usr/local/lib/libfastcdr.so
    /usr/local/lib/libfoonathan_memory-0.7.3.a
    pthread dl
)

# ── Booster SDK targets (inherit link_libraries below) ────────────────────────
# Use explicit /usr/local/lib full paths so the correct FastDDS v2.13 / FastCDR v2
# is always linked, regardless of whether the ROS env is sourced at build time.
link_libraries(
    /usr/local/lib/libbooster_robotics_sdk.a
    /usr/local/lib/libfastrtps.so
    /usr/local/lib/libfastcdr.so
    /usr/local/lib/libfoonathan_memory-0.7.3.a
)

add_executable(b1_loco_example_client example/high_level/b1_loco_example_client.cpp)
add_executable(b1_arm_sdk_example_client example/high_level/b1_arm_sdk_example.cpp)
add_executable(b1_7dof_arm_sdk_example_client example/high_level/b1_7dof_arm_sdk_example.cpp)
add_executable(b1_upper_body_custom_control example/high_level/b1_upper_body_custom_control.cpp)
add_executable(b1_low_level_publisher example/low_level/low_level_publisher.cpp)
add_executable(b1_low_level_subscriber example/low_level/low_level_subscriber.cpp)
add_executable(low_level_hand_data_subscriber example/low_level/low_level_hand_data_subscriber.cpp)
add_executable(b1_low_sdk_example example/low_level/b1_low_sdk_example.cpp)
add_executable(b1_7dof_arm_low_sdk_example example/low_level/b1_7dof_arm_low_sdk_example.cpp)
add_executable(odometer_example example/low_level/odometer_example.cpp)
add_executable(battery_state_subscriber example/low_level/battery_state_subscriber.cpp)


include_directories(BEFORE ${PROJECT_SOURCE_DIR}/third_party/include ${PROJECT_SOURCE_DIR}/include)

if(BUILD_PYTHON_BINDING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
    message(STATUS "Python3 executable: ${Python3_EXECUTABLE}")

    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    include_directories(${Python3_INCLUDE_DIRS})
    find_package(pybind11 CONFIG REQUIRED)

    python3_add_library(booster_robotics_sdk_python MODULE python/binding.cpp WITH_SOABI)

    target_compile_definitions(booster_robotics_sdk_python PRIVATE VERSION_INFO=${PROJECT_VERSION})

    find_program(PYBIND11_STUBGEN_EXECUTABLE pybind11-stubgen)
    if(NOT PYBIND11_STUBGEN_EXECUTABLE)
        message(FATAL_ERROR "pybind11-stubgen not found")
    endif()

    add_custom_command(
        TARGET booster_robotics_sdk_python
        POST_BUILD
        COMMAND PYTHONPATH=${CMAKE_SOURCE_DIR}/build:/${PYTHONPATH} pybind11-stubgen -o ${CMAKE_SOURCE_DIR}/build booster_robotics_sdk_python
    )

    install(TARGETS booster_robotics_sdk_python LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES})
    install(FILES ${CMAKE_SOURCE_DIR}/build/booster_robotics_sdk_python.pyi DESTINATION ${PYTHON_SITE_PACKAGES})    
endif()

