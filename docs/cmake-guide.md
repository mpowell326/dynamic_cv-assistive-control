# Cmake Guide
Run cmake in dir with CMakeLists.txt. Use as
  `cmake [build_path] [optional paramers]` to generate MakeFiles, then run `make` to build.

## Basic use
    cmake_minimum_required (VERSION 2.6)
    project (Tutorial)

Can refer to project name with `${PROJECT}`.
#### Add executables
    add_executable(Executable_name src_files.cxx [additional src_files/libraries])

#### Adding libraries/directories
    include_directories ("DIR_PATH")
    add_subdirectory (name_of_sub_directory)

And to link a library to an executable use:

    target_link_libraries (Executable_name LibraryName)

Collect together various libaries into one variable

    set (EXTRA_LIBS ${EXTRA_LIBS} [MoreLibraries])

#### Build options
    option (USE_MYMATH "Use tutorial provided math implementation" ON)

Set options when running Cmake with `-D` switch

Can then use if statements to adjust Build

    if(USE_MYMATH)
      stuff...
    endif (USE_MYMATH)


#### Finding Packages/libaries
Try `find_package(PackageName [VERSION] [COMPONENTS ...] [REQUIRED])` initially to see if a Find[PackageName].cmake exists. If the package is found you can then use `PACKAGENAME_LIBRARIES` and `PACKAGENAME_INCLUDE_DIRS`

    find_package(PackageName 2.7)

    if(PACKAGENAME_FOUND)
        target_link_libraries (Executable_name ${PACKAGENAME_LIB})
    endif(PACKAGENAME_FOUND)

Or find library manually by doing:

    find_library(GLFW_LIBRARIES NAMES glfw glfw3
        PATHS /usr/lib64
              /usr/lib
              /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
              /usr/local/lib64
              /usr/local/lib
              /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
              /usr/X11R6/lib


Or set library path and include dir:

    set(LibaryName_INCLUDE_DIR path_to_include_dir)
    set( path_library_dir)

    include_directories(${LibaryName_INCLUDE_DIR})
    link_directories(${LibaryName_LIBRARY_DIR})

Refer to [https://cmake.org/cmake-tutorial/] and [http://www.vtk.org/Wiki/CMake/Examples#Link_to_a_library] for further use such as install and testing.
