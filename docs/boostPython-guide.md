# Boost-Python Guide

Use boost libraries to create a python usable library from c/c++ source code.

Refer to [boost website](http://www.boost.org/doc/libs/1_62_0/libs/python/doc/html/tutorial/index.html ) for full tutorial.

## Expose C++ code to python with Boost.Python wrapper
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






## Embedding python code in C++
Going in the reverse direction (python to c++) is not as well supported by boost but can still be done.

Need to include:
    #include <boost/python.hpp>
    #include <Python.h>

#### Setup environment and create class object
```c++
setenv("PYTHONPATH", "/home/morgan/src/unidrive-edit/lib/platform/tools/can/", 1);

/* Initalize python-can interface */
Py_Initialize();

/* Initalize various objects used */
boost::python::object main_module, main_namespace, canCameraInteraface_module, canCameraInteraface,temp;

/* Import main module and namespace */
main_module = boost::python::import("__main__");
assert(main_module);
main_namespace = main_module.attr("__dict__");
assert(main_namespace);

/* Import our python can interface/class */
exec(
       "import camera_can_interface\n"
       "canCameraInteraface = camera_can_interface.CanCameraInteraface()\n"
     , main_namespace
   );

/* Create an instance of the python class in the main namespace */
canCameraInteraface = main_namespace["canCameraInteraface"];

```

#### Execute functions
```c++
/* Get the joystick demands from the CAN bus */
canCameraInteraface.attr("updateJsDemands")();
xDemand = boost::python::extract<int>(   canCameraInteraface.attr("getJsXdemand")()  )();
yDemand = boost::python::extract<int>(   canCameraInteraface.attr("getJsYdemand")()  )();
```

#### Python Garbage Collector
Python will delete objects when it thinks they are no longer needed (with the garbage collector referenced to in the [gc module](https://docs.python.org/2/library/gc.html)). This can be prevented over-riding the default \__del\__() method (or by possibly increasing the objects reference count).


```python
def __del__(self):
    print("trying to delete")
```

http://stackoverflow.com/questions/6315244/how-to-give-object-away-to-python-garbage-collection
http://stackoverflow.com/questions/8025888/does-python-gc-deal-with-reference-cycles-like-this

#### Catch python error-use-of-deleted-function
```c++
try
{
  ...
}
catch( boost::python::error_already_set )
{
    PyErr_Print();
}
```

#### Possible useful:
http://thejosephturner.com/blog/post/embedding-python-in-c-applications-with-boostpython-part-1/
http://members.gamedev.net/sicrane/articles/EmbeddingPythonPart1.html
https://wiki.python.org/moin/boost.python/EmbeddingPython
http://www.boost.org/doc/libs/1_62_0/libs/python/doc/html/tutorial/tutorial/embedding.html
https://wiki.python.org/moin/boost.python/HowTo
https://cs.brown.edu/~jwicks/boost/libs/python/doc/tutorial/doc/html/python/embedding.html
