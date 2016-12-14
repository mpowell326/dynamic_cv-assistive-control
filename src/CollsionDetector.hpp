#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "RsCamera.hpp"
#include "ObjectDetector.hpp"

class CollisionDetector : public ObjectDetector
{
public:
    CollisionDetector();
    ~CollisionDetector();
    
    void start();
    void update();
    void stop();
    
	RsCamera rsCamera;
    ObjectDetector objectDetector;
};





#endif  // COLLISION_DETECTOR_H