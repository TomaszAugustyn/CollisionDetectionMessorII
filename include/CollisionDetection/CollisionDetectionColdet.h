/** @file CollisionDetectionColdet.h
 *
 * implementation - CollisionDetectionColdet
 *
 */

#ifndef COLLISIONDETECTIONCOLDET_H_INCLUDED
#define COLLISIONDETECTIONCOLDET_H_INCLUDED

#include "CollisionDetection.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>

namespace coldet {
	/// create a single grabber (Kinect)
	CollisionDetection* createCollisionDetectionColdet(void);
    CollisionDetection* createCollisionDetectionColdet(std::string configFile);
};

using namespace coldet;

/// CollisionDetection implementation
class CollisionDetectionColdet : public coldet::CollisionDetection {
    public:

        /// Pointer
        typedef std::unique_ptr<CollisionDetectionColdet> Ptr;

        /// Construction
        CollisionDetectionColdet (void);

        /// Construction
        CollisionDetectionColdet(std::string configFilename) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load Kinect config file.\n";
            tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
            double query[4];
            posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
            double queryPos[4];
            posXML->QueryDoubleAttribute("x", &queryPos[0]); posXML->QueryDoubleAttribute("y", &queryPos[1]); posXML->QueryDoubleAttribute("z", &queryPos[2]);
//            pose = Quaternion (query[0], query[1], query[2], query[3])*Vec3(queryPos[0], queryPos[1], queryPos[2]);
        }

        /// Name of the grabber
        const std::string& getName() const;

       
    private:
};

#endif // COLLISIONDETECTIONCOLDET_H_INCLUDED
