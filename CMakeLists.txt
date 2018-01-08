#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_minimum_required(VERSION 3.1.0)

project(test_librealsense)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenGL)
set(DEPENDENCIES realsense2 ${OPENGL_LIBRARIES})

# Find glfw header
find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
    PATHS /usr/X11R6/include
          /usr/include/X11
          /opt/graphics/OpenGL/include
          /opt/graphics/OpenGL/contrib/libglfw
          /usr/local/include
          /usr/include/GL
          /usr/include
)
# Find glfw library
find_library(GLFW_LIBRARIES NAMES glfw glfw3
        PATHS /usr/lib64
              /usr/lib
              /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
              /usr/local/lib64
              /usr/local/lib
              /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
              /usr/X11R6/lib
)
list(APPEND DEPENDENCIES m ${GLFW_LIBRARIES} ${LIBUSB1_LIBRARIES})
include_directories(${GLFW_INCLUDE_DIR})

add_executable (test_librealsense test_librealsense.cpp)
target_link_libraries (test_librealsense ${DEPENDENCIES})
#target_link_libraries (test_librealsense)
#g++ -std=c++11 ../test_librealsense.cpp -lrealsense2 -o test_librealsense
