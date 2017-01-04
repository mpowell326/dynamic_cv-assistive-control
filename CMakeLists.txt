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
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#     message("c11")
# else
if(COMPILER_SUPPORTS_CXX0X)
    message("c0x")
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
#find_library(ARUCO_LIB NAMES libaruco.so REQUIRED
#        PATHS /usr/local/lib )

#include_directories(${ARUCO_INCLUDE_DIR})
#list(APPEND DEPENDENCIES m aruco)


# Add executables and link libraries
# -------------------------------------------------------------------------
# Basic RGB camera display:
# add_executable( bin/DisplayImage src/displayImage.cpp )
# target_link_libraries( bin/DisplayImage ${OpenCV_LIBS} )

# Another basic test:
# add_executable( bin/DisplayVideo src/test1.cpp )
# target_link_libraries( bin/DisplayVideo ${OpenCV_LIBS} )

# Aruco Fiducial Marker Tracker:
# add_executable( bin/Aruco_tracker src/arucoMarkerTracker.cpp src/RgbCamera.cpp)
# target_link_libraries( bin/Aruco_tracker ${OpenCV_LIBS} ${ARUCO_LIB})

# Librealsense example: 
# add_executable( bin/realsenseTest src/main.cpp )
# target_link_libraries( bin/realsenseTest ${OpenCV_LIBS} realsense)

# Main collision avoidance program:
add_executable( bin/collision src/Camera.cpp src/ObjectDetector.cpp src/RsCamera.cpp src/main.cpp )
target_link_libraries( bin/collision ${OpenCV_LIBS} realsense ${DEPENDENCIES} )
  


# link Python wrapper/package with Boost and create libraries
# -------------------------------------------------------------------------
FIND_PACKAGE(Boost 1.54.0)


IF(Boost_FOUND)
  INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "/usr/include/python2.7")
  SET(Boost_USE_STATIC_LIBS OFF)
  SET(Boost_USE_MULTITHREADED ON)
  SET(Boost_USE_STATIC_RUNTIME OFF)
  FIND_PACKAGE(Boost 1.54.0 COMPONENTS python)

  # find_library( BOOST_PYTHON NAMES libboost_python* libboost-python*
  #       PATHS /usr/lib/x86_64-linux-gnu/
  #             /usr/lib64
  #             /usr/lib
  #             /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
  #             /usr/local/lib64
  #             /usr/local/lib
  #             /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
  #             )



  IF(Boost_FOUND)
    # ADD_LIBRARY(_arucoTracker SHARED src/arucoMarkerTracker.cpp src/RgbCamera.cpp)
    # TARGET_LINK_LIBRARIES(_arucoTracker ${Boost_LIBRARIES}  ${ARUCO_LIB} ${OpenCV_LIBS}) #${BOOST_PYTHON}

    ADD_LIBRARY(_collision_ext SHARED src/Camera.hpp src/RsCamera.cpp src/ObjectDetector.cpp src/CollsionDetector.cpp src/collision_ext.cpp)
    TARGET_LINK_LIBRARIES(_collision_ext ${Boost_LIBRARIES}  ${OpenCV_LIBS} realsense ${DEPENDENCIES} )


  ELSEIF(NOT Boost_FOUND)
    MESSAGE(FATAL_ERROR "Unable to find  Boost python")
  ENDIF()

ELSEIF(NOT Boost_FOUND)
  MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
ENDIF()

IF(CMAKE_COMPILER_IS_GNUCXX)
  ADD_DEFINITIONS("-Wall")
ELSE()
  MESSAGE(FATAL_ERROR "CMakeLists.txt has not been tested/written for your compiler.")
ENDIF()

