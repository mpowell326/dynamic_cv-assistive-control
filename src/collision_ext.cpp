
#ifndef COLLISION_EXT_H
#define COLLISION_EXT_H


#include <Python.h>
#include <boost/python.hpp>
// #include "RsCamera.hpp"
#include "CollsionDetector.hpp"

template<typename T>
struct custom_vvector_to_list{
    static PyObject* convert(const std::vector<std::vector<T> > vv){
        boost::python::list ret;

        BOOST_FOREACH(const std::vector<T>& v, vv)
        {
            boost::python::list ret2;
            BOOST_FOREACH(const T& e, v) 
            {
                boost::python::make_tuple(e.first, e.second);
                ret2.append(boost::python::make_tuple(e.first, e.second));
            }
            ret.append(ret2);
        }
        return boost::python::incref(ret.ptr());
    }
};

template<typename T>
struct custom_vvector_int_to_list{
    static PyObject* convert(const std::vector<std::vector<T> > vv){
        boost::python::list ret;

        BOOST_FOREACH(const std::vector<T>& v, vv)
        {
            boost::python::list ret2;
            BOOST_FOREACH(const T& e, v) 
            {
                ret2.append(e);
            }
            ret.append(ret2);
        }
        return boost::python::incref(ret.ptr());
    }
};


template<typename T>
struct custom_vector_to_list{
    static PyObject* convert(const std::vector<T> v){
        boost::python::list ret;

        BOOST_FOREACH(const T& e, v) 
        {
            boost::python::make_tuple(e.first, e.second);
            ret.append(boost::python::make_tuple(e.first, e.second));
        }
        return boost::python::incref(ret.ptr());
    }
};


// template<typename T>
// boost::python::list toPythonList(std::vector<std::vector<T> > vv) {
//     typename std::vector<T>::iterator iter;
    
//     boost::python::list list;

//     BOOST_FOREACH(const std::vector<T>& v, vv)
//     {
//         boost::python::list ret2;
//         BOOST_FOREACH(const T& e, v)
//         {
//             ret2.append(e);
//         }
//         list.append(ret2);
//     }

//     return list;
// }


// Converts a C++ vector to a python list
// template <class T>
// boost::python::list toPythonList(std::vector<T> vector) {
//     typename std::vector<T>::iterator iter;
//     boost::python::list list;
//     for (iter = vector.begin(); iter != vector.end(); ++iter) {
//         list.append(*iter);
//     }
//     return list;
// }


BOOST_PYTHON_MODULE(lib_collision_ext)
{   
    using namespace boost::python;


    // custom_vvector_to_list<std::pair<double, double>>();
    // to_python_converter<std::pair<int, int>, PairToTupleConverter<int, int> >();
    boost::python::to_python_converter<   std::vector<std::vector<std::pair<double, double> > >     , custom_vvector_to_list<std::pair<double, double>>      >();
    custom_vvector_to_list<std::pair<double, double>>  ();

    boost::python::to_python_converter<   std::vector<std::vector<int > >     , custom_vvector_int_to_list<int>      >();
    custom_vvector_int_to_list<int>  ();


    boost::python::to_python_converter<   std::vector<std::pair<int, int> >      , custom_vector_to_list<std::pair<int, int>>      >();
    custom_vvector_to_list<std::pair<int, int>>  ();


    class_<CollisionDetector>("CollisionDetector",init<>())
        .def("start", &CollisionDetector::start)
        .def("update", &CollisionDetector::update)
        .def("stop", &CollisionDetector::stop)
        .def_readwrite("isClose", &CollisionDetector::isClose)
        // .def_readwrite("obstacle_map", &CollisionDetector::obstacle_map);
        .add_property("obstacle_map", make_getter(&CollisionDetector::obstacle_map, return_value_policy<return_by_value>()))
        .def_readwrite("mapWidth", &CollisionDetector::mapWidth)
        .def_readwrite("mapDepth", &CollisionDetector::mapDepth);
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