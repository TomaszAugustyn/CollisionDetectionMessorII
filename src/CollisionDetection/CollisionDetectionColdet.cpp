#include "../include/CollisionDetection/CollisionDetectionColdet.h"
#include <memory>
#include <stdexcept>
#include "../include/CollisionDetection/3dsloader.h"
//#include <chrono>
//#include <thread>

using namespace coldet;

/// A single instance of Kinect grabber
CollisionDetectionColdet::Ptr grabberKinect;

CollisionDetectionColdet::CollisionDetectionColdet(void) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET) {

}

const std::string& CollisionDetectionColdet::getName() const {
	return name;
}


CollisionDetection* coldet::createCollisionDetectionColdet(void) {
    grabberKinect.reset(new CollisionDetectionColdet());
    return grabberKinect.get();
}

CollisionDetection* coldet::createCollisionDetectionColdet(std::string configFile) {
    grabberKinect.reset(new CollisionDetectionColdet(configFile));
    return grabberKinect.get();
}

