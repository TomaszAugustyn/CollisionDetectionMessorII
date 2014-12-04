/** @file CollisionDetection.h
 *
 * Point Cloud CollisionDetection interface
 */
#ifndef _COLLISIONDETECTION_H_
#define _COLLISIONDETECTION_H_

#include "../Defs/defs.h"
#include <iostream>
#include <string>
#include <vector>
#include "../include/CollisionDetection/3dsloader.h"
#include "../include/CollisionDetection/types.h"
#include "../include/CollisionDetection/coldet.h"
#include "../include/CollisionDetection/objects3DS.h"
#include "../include/CollisionDetection/CQuaternion.h"
#include "../include/CollisionDetection/Punctum.h"
#include <GL/glut.h>
//#include <mutex>

namespace coldet{
	//CollisionDetection interface
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

			/// overloaded constructor
            CollisionDetection(const std::string _name, Type _type) : name(_name), type(_type) {};

			/// Name of the structure
			virtual const std::string& getName() const {
			return name;
			}

			/// Initialize robot structure
			virtual void initStructures(void) = 0;

			/// Draw robot using openGL
			virtual void GLDrawRobot(coldet::float_type *pos, coldet::float_type * rot, std::vector<coldet::float_type> config) const = 0;

			/// Check collisions
			virtual bool checkCollision(coldet::float_type* pos, coldet::float_type* rot, coldet::float_type * angles, bool * collision_table) const = 0;

			/*void CheckCollisions (const RobotConfiguration& config, CollisionTable& collisionTable);
			void DecodeCollisionTable (const CollisionTable ...);
			void CheckCollisions (const Mat34 RobotPose, const RobotConfiguration config, const CElevationMap& map, CollisionTable& collisionTable);

			bool CheckCollisions(const RobotConfiguration& config, CollisionTable& collisionTable);
			bool CheckCollisions (const Mat34 RobotPose, const RobotConfiguration config, const CElevationMap& map, CollisionTable& collisionTable);
			void drawGL (const Mat34 RobotPose, const RobotConfiguration config); */

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
