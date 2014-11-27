/** @file CollisionDetection.h
 *
 * Point Cloud CollisionDetection interface
 *
 */

#ifndef _COLLISIONDETECTION_H_
#define _COLLISIONDETECTION_H_

#include "../Defs/defs.h"
#include <iostream>
#include <string>
#include <vector>
#include "../include/CollisionDetection/3dsloader.h"
#include "../include/CollisionDetection/types.h"
//#include <mutex>

namespace coldet{
	/// CollisionDetection interface
	class CollisionDetection {
        public:

            /// CollisinDetection type
            enum Type {
                    /// RGB camera
                    TYPE_COLDET,
                    /// 2D Depth sensor
                    TYPE_VOXEL,
                    /// 3D Depth sensor
            };

			/*void CheckCollisions (const RobotConfiguration& config, CollisionTable& collisionTable);
			void DecodeCollisionTable (const CollisionTable ...);
			void CheckCollisions (const Mat34 RobotPose, const RobotConfiguration config, const CElevationMap& map, CollisionTable& collisionTable);

			bool CheckCollisions(const RobotConfiguration& config, CollisionTable& collisionTable);
			bool CheckCollisions (const Mat34 RobotPose, const RobotConfiguration config, const CElevationMap& map, CollisionTable& collisionTable);
			void drawGL (const Mat34 RobotPose, const RobotConfiguration config); */
			
			//Loads .3ds object 
			char Load3DS (obj_type_ptr p_object, const char *p_filename);

            /// overloaded constructor
            CollisionDetection(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the CollisionDetection
            virtual const std::string& getName() const = 0;

            /// Virtual descrutor
            virtual ~CollisionDetection() {}

        protected:
            /// CollisonDetection type
            Type type;

            /// CollisonDetection name
            const std::string name;
	};
};

#endif // _COLLISIONDETECTION_H_
