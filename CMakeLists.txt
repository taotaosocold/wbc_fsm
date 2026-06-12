# 指定构建该项目所需的最低 CMake 版本为 3.14
cmake_minimum_required(VERSION 3.14)
# 定义项目名称为 g1_onnx_rl
project(g1_onnx_rl)
# 设置构建类型为Release，cmake有四种内置构建类型，Debug和Release和RelWithDebInfo和MinSizeRel
set(CMAKE_BUILD_TYPE Release)
# 要求 C++17 标准
set(CMAKE_CXX_STANDARD 17)
# 将 CMAKE_CXX_STANDARD_REQUIRED 设为 ON，表示如果编译器不支持 C++17 则会报错终止，而不是退而求其次使用更低版本
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 将 /opt/unitree_robotics 追加到 CMAKE_PREFIX_PATH 列表中（在配置unitree_sdk2的环境的时候，会把所需的库编译好拷贝到/opt/unitree_robotics下）
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")
# 搜索系统中的 Boost C++ 库
find_package(Boost)
# 找 nlohmann_json 这个 JSON 解析库，版本必须 >= 3.7.3
find_package(nlohmann_json 3.7.3 REQUIRED)
# 找 Eigen3 线性代数库
find_package(Eigen3 REQUIRED)
# 找unitree_sdk2这个库（因为前面已经将库添加到了CMAKE_PREFIX_PATH路径下了，所以这里找的到）
find_package(unitree_sdk2 REQUIRED)
# 至此内部就会维护一张这样的表格：
# PROJECT_SOURCE_DIR           │ /home/casbot/Desktop/wbc_fsm
# CMAKE_CXX_STANDARD           │ 17
# CMAKE_BUILD_TYPE             │ Release
# Boost_INCLUDE_DIRS           │ /usr/include
# Boost_LIBRARIES              │ /usr/lib/.../libboost_program_options.so
# nlohmann_json_DIR            │ /usr/lib/cmake/nlohmann_json
# Eigen3_INCLUDE_DIRS          │ /usr/include/eigen3
# unitree_sdk2_INCLUDE_DIRS    │ /opt/unitree_robotics/include
# unitree_sdk2_LIBRARIES       │ /opt/unitree_robotics/lib/x86_64/lib...

# 这里很重要了，这里自定义一个变量ONNXRUNTIME_ROOT指向项目目录下的ONNX Runtime解压包（readme中有去下载onnx压缩包并解压）
# 这里有两个不同的解压包一个是用于仿真一个是用于实机
set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-x64-1.22.0)
# set(ONNXRUNTIME_ROOT ${PROJECT_SOURCE_DIR}/onnxruntime-linux-aarch64-1.22.0)
# 设置头文件搜索路径。每个路径相当于给 g++ 传一个 -I 参数
include_directories(
    include
    ${ONNXRUNTIME_ROOT}/include
    ${ONNXRUNTIME_ROOT}/include/onnxruntime
    ${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session
    ${Boost_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenSSL_INCLUDE_DIR}
)
# 给 C++ 编译器追加编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")
# 添加库文件搜索路径。相当于 g++ -L onnxruntime-linux-x64-1.22.0/lib
link_directories(${ONNXRUNTIME_ROOT}/lib)
# 自动收集 src/ 下所有子目录里的 .cpp 和 .cc 文件，放入 SRC_LIST 变量
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)
# 添加一个由叫做wbc_fsm的可执行文件
add_executable(wbc_fsm src/main.cpp ${SRC_LIST})
# 给 wbc_fsm 这个目标的所有源文件，在编译时定义一个宏 PROJECT_ROOT_DIR
target_compile_definitions(wbc_fsm PRIVATE 
    PROJECT_ROOT_DIR="${PROJECT_SOURCE_DIR}"
)

# 链接阶段，把编译好的 wbc_fsm 的 .o 文件和这些外部库拼成一个可执行文件wbc_fsm
target_link_libraries(wbc_fsm PRIVATE
    ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so.1.22.0
    unitree_sdk2
    ${CUDA_LIBRARIES}
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)