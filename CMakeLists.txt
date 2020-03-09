cmake_minimum_required(VERSION 3.15)
project(CylinderDetection)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

# visualisation
find_package(glfw3 3.2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 14)

add_executable(CylinderDetection src/main.cpp src/opengl_helper.cpp src/opengl_helper.h src/cylinder_detection.cpp src/cylinder_detection.h src/CameraLocation.cpp src/constants.h src/AABB.cpp src/AABB.h)

# OpenCV


# Libs
target_include_directories(CylinderDetection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CylinderDetection PUBLIC ${OpenCV_LIBS} realsense2 OpenGL::GL OpenGL::GLU glfw ${GLFW_STATIC_LIBRARIES} pthread Eigen3::Eigen)
