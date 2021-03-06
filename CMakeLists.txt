cmake_minimum_required(VERSION 3.15)
project(CylinderDetection)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

# visualisation
find_package(glfw3 3.2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 17)

set(SRC src/opengl_helper.cpp src/cylinder_detection.cpp src/cylinder_detection.h src/CameraLocation.cpp src/AABB.cpp src/run.cpp src/network/Client.cpp)
add_executable(CylinderDetectionGUI src/main.cpp ${SRC})
target_compile_definitions(CylinderDetectionGUI PUBLIC USE_GUI)
add_executable(CylinderDetectionNoGUI src/main.cpp ${SRC})
add_executable(Recorder src/record/main.cpp ${SRC})
add_executable(Playback src/playback/main.cpp ${SRC})
target_compile_definitions(Playback PUBLIC USE_GUI)

# OpenCV


# Libs
target_include_directories(CylinderDetectionGUI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CylinderDetectionGUI PUBLIC ${OpenCV_LIBS} realsense2 OpenGL::GL OpenGL::GLU glfw ${GLFW_STATIC_LIBRARIES} pthread Eigen3::Eigen)
target_include_directories(CylinderDetectionNoGUI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(CylinderDetectionNoGUI PUBLIC ${OpenCV_LIBS} realsense2 ${GLFW_STATIC_LIBRARIES} pthread Eigen3::Eigen)

target_include_directories(Recorder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Recorder PUBLIC ${OpenCV_LIBS} realsense2 OpenGL::GL OpenGL::GLU glfw ${GLFW_STATIC_LIBRARIES} pthread Eigen3::Eigen)

target_include_directories(Playback PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Playback PUBLIC ${OpenCV_LIBS} realsense2 OpenGL::GL OpenGL::GLU glfw ${GLFW_STATIC_LIBRARIES} pthread Eigen3::Eigen)
