#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "RsCamera.hpp"
#include "ObjectDetector.hpp"

class CollisionDetector
{
public:
    CollisionDetector() : rsCamera(), objectDetector() {}
    ~CollisionDetector();
    
    void start();
    void update();
    void stop();
    
	RsCamera rsCamera;
    ObjectDetector objectDetector;

    bool isClose;
};



#endif  // COLLISION_DETECTOR_H