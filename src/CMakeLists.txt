cmake_minimum_required(VERSION 3.28)
project(stardust)

set(CMAKE_CXX_STANDARD 17)

include_directories(../include ../libs/stb_image ../libs/assimp/include)

add_library(stardust sd_memory.cpp
        sd_win32.cpp
        sd_math.cpp
        sd_input.cpp
        sd_software_renderer.cpp
        sd_mesh.cpp
        sd_animation.cpp
        sd_physics.cpp
        sd_collision.cpp)

project(sample)
set(CMAKE_CXX_STANDARD 17)

include_directories(../include)
link_directories(cmake-build-debug ../libs/assimp/bin)
link_libraries(stardust.lib assimp-vc143-mt.lib)

add_executable(sample main.cpp)