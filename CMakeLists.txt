cmake_minimum_required(VERSION 3.5)
project(Bachelor)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_subdirectory(visualizer)
add_subdirectory(astar)

add_executable(Bachelor main.cpp )
target_link_libraries(Bachelor visualizer PP)
add_custom_command(
    TARGET Bachelor
    POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        $<TARGET_FILE_DIR:Bachelor>/assets)





