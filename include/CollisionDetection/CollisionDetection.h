/** @file CollisionDetection.h
 *
 * Point Cloud Grabber interface
 *
 */

#ifndef _COLLISIONDETECTION_H_
#define _COLLISIONDETECTION_H_

#include "../Defs/defs.h"
#include <iostream>
#include <string>
#include <vector>
//#include <mutex>

namespace coldet{
	/// Grabber interface
	class CollisionDetection {
        public:

            /// Grabber type
            enum Type {
                    /// RGB camera
                    TYPE_COLDET,
                    /// 2D Depth sensor
                    TYPE_VOXEL,
                    /// 3D Depth sensor
            };

            /// overloaded constructor
            CollisionDetection(const std::string _name, Type _type) : name(_name), type(_type) {};

            /// Name of the grabber
            virtual const std::string& getName() const = 0;

           

            /// Virtual descrutor
            virtual ~CollisionDetection() {}

        protected:
            /// Grabber type
            Type type;

            /// Grabber name
            const std::string name;
	};
};

#endif // _GRABBER_H_
