cmake_minimum_required(VERSION 3.14)
project(casbot_wbc_fsm)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(nlohmann_json 3.7.3 REQUIRED)
find_package(Eigen3 REQUIRED)

set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)

include_directories(
    include
    ${ONNXRUNTIME_ROOT}/include
    ${ONNXRUNTIME_ROOT}/include/onnxruntime
    ${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session
    ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")

link_directories(${ONNXRUNTIME_ROOT}/lib)

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(wbc_fsm src/main.cpp ${SRC_LIST})

set_target_properties(wbc_fsm PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib:/opt/ros/humble/lib"
    INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib:/opt/ros/humble/lib"
    BUILD_WITH_INSTALL_RPATH FALSE
    SKIP_BUILD_RPATH FALSE
)

target_compile_definitions(wbc_fsm PRIVATE
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

ament_target_dependencies(wbc_fsm
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
)

target_link_libraries(wbc_fsm
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so.1.22.0
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)

# Fix RPATH to exclude conda library paths
add_custom_command(TARGET wbc_fsm POST_BUILD
    COMMAND patchelf --set-rpath
    "${ONNXRUNTIME_ROOT}/lib:/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu"
    $<TARGET_FILE:wbc_fsm>
    COMMENT "Fixing RPATH for wbc_fsm"
)

install(TARGETS wbc_fsm
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
