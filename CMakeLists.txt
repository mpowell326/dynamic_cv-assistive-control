# cmake list for computer vision assisted wheelchair project
project( CVAssist )

# Check for cmake version
# -------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8)


# Set compiler settings/add dependencies needed for librealsense
# -------------------------------------------------------------------------
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenGL REQUIRED)
set(DEPENDENCIES realsense ${OPENGL_LIBRARIES})

if(WIN32)
    add_subdirectory(third_party/glfw)
    list(APPEND DEPENDENCIES glfw3)
else()
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
endif()


# Add librealsense library
# -------------------------------------------------------------------------
find_library( REALSENSE_LIBRARIES NAMES librealsense.so 
        PATHS /usr/local/lib)


# Add openCV package
# -------------------------------------------------------------------------
find_package( OpenCV REQUIRED )


# Add Aruco library used for marker tracking/detection
# -------------------------------------------------------------------------
find_library(ARUCO_LIB NAMES libaruco.so REQUIRED
        PATHS /usr/local/lib )

# include_directories(${ARUCO_INCLUDE_DIR})
# list(APPEND DEPENDENCIES m aruco)


# Add executables and link libraries
# -------------------------------------------------------------------------
add_executable( bin/DisplayImage src/displayImage.cpp )
target_link_libraries( bin/DisplayImage ${OpenCV_LIBS} )


add_executable( bin/DisplayVideo src/test1.cpp )
target_link_libraries( bin/DisplayVideo ${OpenCV_LIBS} )


add_executable( bin/CVAssist src/main.cpp )
target_link_libraries( bin/${PROJECT_NAME} ${OpenCV_LIBS} realsense)

  
add_executable( bin/Aruco_tracker src/arucoMarkerTracker.cpp )
target_link_libraries( bin/Aruco_tracker ${OpenCV_LIBS} ${ARUCO_LIB} )





