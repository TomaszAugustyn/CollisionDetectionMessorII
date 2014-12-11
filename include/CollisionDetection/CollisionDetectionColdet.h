/** @file CollisionDetectionColdet.h
 *
 * implementation - CollisionDetectionColdet
 */
#ifndef COLLISIONDETECTIONCOLDET_H_INCLUDED
#define COLLISIONDETECTIONCOLDET_H_INCLUDED

#include "../include/CollisionDetection/CollisionDetection.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <memory>

using namespace coldet;

namespace coldet {
	/// create a single CollisionDetectionColdet
	CollisionDetection* createCollisionDetectionColdet(void);
    CollisionDetection* createCollisionDetectionColdet(std::string configFile);
};

/// CollisionDetection implementation
class CollisionDetectionColdet : public coldet::CollisionDetection {
    public:

		//**********OpenGL Call Lists*********************
		static const uint_fast8_t GL_PLATFORM = 1; 
		static const uint_fast8_t GL_COXA = 2;
		static const uint_fast8_t GL_FEMUR = 3;
		static const uint_fast8_t GL_VITULUS = 4;
		//*********************************************

		enum MechParts
		{
			PLATFORM, //0
			VITULUS1, VITULUS2, VITULUS3, VITULUS4, VITULUS5, VITULUS6,//1,2,3,4,5,6
			FEMUR1, FEMUR2, FEMUR3, FEMUR4, FEMUR5, FEMUR6,//7,8,9,10,11,12
			COXA1, COXA2, COXA3, COXA4, COXA5, COXA6,//13,14,15,16,17,18,19
		};
        /// Pointer
        typedef std::unique_ptr<CollisionDetectionColdet> Ptr;

        /// Constructor
        CollisionDetectionColdet (void);

        /// Overloaded Constructor
        CollisionDetectionColdet(std::string configFilename) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load config file.\n";
            tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
            double query[4];
            posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
            double queryPos[4];
            posXML->QueryDoubleAttribute("x", &queryPos[0]); posXML->QueryDoubleAttribute("y", &queryPos[1]); posXML->QueryDoubleAttribute("z", &queryPos[2]);
			//pose = Quaternion (query[0], query[1], query[2], query[3])*Vec3(queryPos[0], queryPos[1], queryPos[2]);
        }

        /// Name of the CollisionDetectionColdet
        const std::string& getName() const;

		/// Destructor
		~CollisionDetectionColdet (void);

		/// Draw robot using openGL
		void GLDrawRobot(coldet::float_type *pos, coldet::float_type * rot, std::vector<coldet::float_type> config) const;

		/// Check collisions
		bool checkCollision(coldet::float_type* pos, coldet::float_type* rot, std::vector<coldet::float_type> config, bool * collision_table) const;

       
    private:
		/// Initialize robot structure
		void initStructures(void);
		/// initialize collision model
		void initCollisionModel(uint_fast8_t objectNo, CollisionModel3D& model);
		/// initialize collision models
		void CollisionModels(void);
		/// initialize GL lists
		void structPlatform(void);
		void structCoxa(void);
		void structFemur(void);
		void structVitulus(void);
		void drawCoordinateSystem(void);
		void Leg1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
		void Leg2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
		void Leg3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
		void Leg4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
		void Leg5(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
		void Leg6(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const;
		void GLLeg1(float Qn_1, float Qn_2, float Qn_3) const;
		void GLLeg2(float Qn_1, float Qn_2, float Qn_3) const;
		void GLLeg3(float Qn_1, float Qn_2, float Qn_3) const;
		void GLLeg4(float Qn_1, float Qn_2, float Qn_3) const;
		void GLLeg5(float Qn_1, float Qn_2, float Qn_3) const;
		void GLLeg6(float Qn_1, float Qn_2, float Qn_3) const;
		void copyTable(CPunctum * src, float * dest) const;
		void DrawRobot(coldet::float_type* pos, coldet::float_type* rot, std::vector<coldet::float_type> config) const;

		std::vector<CollisionModel3D*> meshModel;
		//zadane katy dla serwomechanizmow
		//float angles[18];
		///model 3DS
		CObjects3DS robot_model;

};

#endif // COLLISIONDETECTIONCOLDET_H_INCLUDED
