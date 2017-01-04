# Boost-Python Guide

Use boost libraries to create a python usable library from c/c++ source code.

Refer to [boost website](http://www.boost.org/doc/libs/1_62_0/libs/python/doc/html/tutorial/index.html ) for full tutorial.

#### Expose C++ code to python with Boost.Python wrapper
To expose a C++ class need to add a wrapper / BOOST_PYTHON_MODULE. This can be in a seperate file or added to the C++ header file.

Need to the following includes to your .cxx file:
```c++
#include <Python.h>
#include <boost/python.hpp>
```
The wrapper can then be added as follows:
```c++
BOOST_PYTHON_MODULE(lib_MODULENAME) # MODULENAME needs to be the same as the created .so libary
{
    using namespace boost::python;

    class_<World>("World",init<>()) # Class with basic constrcutor
        .def(init<int, float>())    # Second class constructor
        .def("greet", &World::greet)
        .def("set", &World::set)
    ;
}
```
and to expose data members:
```c++
class_<Var>("Var", init<std::string>())
    .def_readonly("name", &Var::member1)
    .def_readwrite("value", &Var::member2)
  ;
```

#### Compiling the library
To create the python library must first make sure appropriate boost libraries are installed.

    libboost1.54.0
    libboost-python1.54.0
    [possibly more ?]

To creat the library in cmake:
 - find boost libraries
 - link files as before for adding an executable but use 'ADD_LIBRARY' instead
 - Make sure to include ${Boost_LIBRARIES} in the target_link_libraries step

```cmake
FIND_PACKAGE(Boost 1.54.0)

IF(Boost_FOUND)
    INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "/usr/include/python2.7")
    SET(Boost_USE_STATIC_LIBS OFF)
    SET(Boost_USE_MULTITHREADED ON)
    SET(Boost_USE_STATIC_RUNTIME OFF)
    FIND_PACKAGE(Boost 1.54.0 COMPONENTS python)  # or try 'find_library'

    IF(Boost_FOUND)
        ADD_LIBRARY(_MODULENAME SHARED src/r.cpp )
        TARGET_LINK_LIBRARIES(_MODULENAME ${Boost_LIBRARIES}  [${Other libraries to be linked}])

    ELSEIF(NOT Boost_FOUND)
        MESSAGE(FATAL_ERROR "Unable to find  Boost python")
    ENDIF()

ELSEIF(NOT Boost_FOUND)
    MESSAGE(FATAL_ERROR "Unable to find correct Boost version. Did you set BOOST_ROOT?")
ENDIF()
```

__NOTE:__   The BOOST_PYTHON_MODULE() name must match the name of the .so library you generate and import into python. (Else you get the error: 'boost_python import error: module does not define init function' )

When building the .so library 'lib' is automatically added to the filename so name the shared library as '\_MODULENAME' to match that specified in the BOOST_PYTHON_MODULE().

It was also found that when compiling with boost-python and librealsense the member 'rs::context rsCtx' of class RsCamera produced an error. This was fixed by storing rsCtx as a pointer rather than a direct member of the class.
[(possibly relevant to problem)](http://stackoverflow.com/questions/19883092/error-implicitly-deleted-because-the-default-definition-would-be-ill-formed-ve) or possibly [(something about C++0xand gcc version )](http://stackoverflow.com/questions/5966698/error-use-of-deleted-function#5966859)

--> SOLUTION: commented out line 288 in /usr/local/include/librealsense/rs.hpp `// context(const context &) = delete;`

#### Using the complied library in python
Import the 'MODULENAME.so' libary to python and then use as regular python library

```python
import lib_MODULENAME

class_instance = lib_MODULENAME.classinit()
```


##### Possibly useful:
- https://wiki.python.org/moin/boost.python/HowTo
- use `nm -D lib_<modulename>.so | grep <modulename>` to give you the name of your init functinowithin the library
