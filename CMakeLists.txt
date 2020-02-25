cmake_minimum_required(VERSION 3.15)
project(CylinderDetection)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)

# visualisation
find_package(glfw3 3.2 REQUIRED)
find_package(OpenGL REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(shape-fitting)

include_directories(include
    ${PCL_INCLUDE_DIRS}
)


add_executable(CylinderDetection src/main.cpp src/opengl_helper.cpp src/opengl_helper.h src/cylinder_detection.cpp src/cylinder_detection.h src/CameraLocation.cpp src/constants.h)


# FITTING LIB
add_library (cylinder_fitting_library shape-fitting/src/cylinder_fitting.cpp shape-fitting/src/cylinder_fitting_hough.cpp shape-fitting/src/gaussian_sphere.cpp shape-fitting/src/fitting_data.cpp)
target_include_directories(cylinder_fitting_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} shape-fitting/include)
target_link_libraries (cylinder_fitting_library ${PCL_LIBRARIES})
#

target_link_libraries(CylinderDetection PUBLIC realsense2 cylinder_fitting_library glfw OpenGL::GL OpenGL::GLU)
target_include_directories(CylinderDetection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} shape-fitting/include)
