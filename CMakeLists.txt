cmake_minimum_required(VERSION 3.15)
project(CylinderDetection)

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(shape-fitting)

include_directories(include
    ${PCL_INCLUDE_DIRS}
)

add_executable(CylinderDetection src/main.cpp)

# FITTING LIB
add_library (cylinder_fitting_library shape-fitting/src/cylinder_fitting.cpp shape-fitting/src/cylinder_fitting_hough.cpp shape-fitting/src/gaussian_sphere.cpp shape-fitting/src/fitting_data.cpp)
target_include_directories(cylinder_fitting_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} shape-fitting/include)
target_link_libraries (cylinder_fitting_library ${PCL_LIBRARIES})
#

target_link_libraries(CylinderDetection PUBLIC cylinder_fitting_library)
target_include_directories(CylinderDetection PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} shape-fitting/include)