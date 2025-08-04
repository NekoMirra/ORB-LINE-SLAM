project( line-descriptor )

cmake_minimum_required(VERSION 2.7)
find_package( OpenCV 3 REQUIRED)

if(COMMAND cmake_policy)
  	cmake_policy(SET CMP0003 NEW)	
endif(COMMAND cmake_policy)
link_directories(${OpenCV_LIBS_DIR})
include_directories(${OpenCV2_INCLUDE_DIRS})

SET(BUILD_SHARED_LIBS ON)
SET(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -mtune=native -march=native")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH    ${PROJECT_SOURCE_DIR}/lib)

include_directories( include ${OpenCV_INCLUDE_DIRS} )
list(APPEND LINK_LIBS ${OpenCV_LIBS} )
file(GLOB_RECURSE all_include_files RELATIVE "${CMAKE_SOURCE_DIR}" *.h *.hpp)

link_directories(${CMAKE_SOURCE_DIR}/src/)

# 手动指定库源文件，排除示例程序
set(LIBRARY_SOURCES
    src/ED_Lib/ED.cpp
    src/ED_Lib/EDCircles.cpp
    src/ED_Lib/EDColor.cpp
    src/ED_Lib/EDLines.cpp
    src/ED_Lib/EDPF.cpp
    src/ED_Lib/NFA.cpp
    src/LSDDetector_custom.cpp
    src/LineGraphModel.cpp
    src/binary_descriptor_custom.cpp
    src/binary_descriptor_matcher.cpp
    src/draw_custom.cpp
)

add_custom_target( linedesc_includes DEPENDS ${all_include_files} SOURCES ${LIBRARY_SOURCES} )

add_library( linedesc ${LIBRARY_SOURCES} )
target_link_libraries( linedesc ${LINK_LIBS} )

# 添加概率图建模示例程序
add_executable( line_graph_example src/line_graph_example.cpp src/LineGraphModel.cpp src/ED_Lib/NFA.cpp )
target_link_libraries( line_graph_example ${LINK_LIBS} )
target_include_directories( line_graph_example PRIVATE include )

# 添加自适应特征提取示例程序
add_executable( adaptive_feature_example src/adaptive_feature_example.cpp src/LineGraphModel.cpp src/ED_Lib/NFA.cpp )
target_link_libraries( adaptive_feature_example ${LINK_LIBS} )
target_include_directories( adaptive_feature_example PRIVATE include )

