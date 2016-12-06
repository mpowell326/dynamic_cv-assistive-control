#include <Python.h>
#include <boost/python.hpp>
// #include <boost/python/detail/wrap_python.hpp>
#include "arucoMarkerTracker.hpp"
using namespace boost::python;

BOOST_PYTHON_MODULE(lib_arucoTracker)
{
    // boost::python::numeric::array::set_module_and_type("numpy", "ndarray");
    
    class_<arucoTracker>("arucoTracker",init<>())
    	.def(init<int, float>())
        .def("detectMarkers", &arucoTracker::detectMarkers)
     	.def("getMarkerPoseXYZ", &arucoTracker::getMarkerPoseXYZ)  
     	.def("displayMarker", &arucoTracker::displayMarker)  
     	.def("setDetectorParams", &arucoTracker::setDetectorParams)
     	.def("getTrackedMarkerID", &arucoTracker::getTrackedMarkerID)
     	.def("openCaptureDevice", &arucoTracker::openCaptureDevice)
     	.def("calibrateFromFile", &arucoTracker::calibrateFromFile)
        .def("getNextFrame", &arucoTracker::getNextFrame)
        .def("getMarkerPoseX", &arucoTracker::getMarkerPoseX)
        .def("getMarkerPoseY", &arucoTracker::getMarkerPoseY)
        .def("getMarkerPoseZ", &arucoTracker::getMarkerPoseZ)
    ;
}


