
#ifndef COLLISION_EXT_H
#define COLLISION_EXT_H


#include <Python.h>
#include <boost/python.hpp>
// #include "RsCamera.hpp"
#include "CollsionDetector.hpp"


BOOST_PYTHON_MODULE(lib_collision_ext)
{   
    using namespace boost::python;

    class_<CollisionDetector>("CollisionDetector",init<>())
        .def("start", &CollisionDetector::start)
        .def("update", &CollisionDetector::update)
        .def("stop", &CollisionDetector::stop)
        .def_readwrite("isClose", &CollisionDetector::isClose);
    ;
}


// BOOST_PYTHON_MODULE(lib_collision_ext)
// {   
//     using namespace boost::python;

//     class_<RsCamera>("RsCamera",init<>())
//         .def("initializeStreaming", &RsCamera::initializeStreaming)
//         .def("setupWindows", &RsCamera::setupWindows)
//         .def("getNextFrame", &RsCamera::getNextFrame)
//         .def("displayStreams", &RsCamera::displayStreams)
//         .def("displayFPS", &RsCamera::displayFPS)
//     ;

//     class_<ObjectDetector, bases<RsCamera>>("ObjectDetector",init<>())
//         .def("isObjectClose", &ObjectDetector::isObjectClose)
//         .def_readwrite("camera", &ObjectDetector::camera)
//     ;
// }

#endif  //COLLISION_EXT_H