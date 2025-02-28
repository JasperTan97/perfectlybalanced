cmake_minimum_required(VERSION 3.10)
project(HelloMuJoCo)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)

include_directories(/mujoco/mujoco-3.3.0/include)
link_directories(/mujoco/mujoco-3.3.0/lib)

add_executable(hello_mujoco hello_mujoco.cpp)
target_link_libraries(hello_mujoco mujoco OpenGL::GL glfw GLEW X11)
