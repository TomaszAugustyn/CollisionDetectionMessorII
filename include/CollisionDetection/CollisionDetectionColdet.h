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
			COXA1, COXA2, COXA3, COXA4, COXA5, COXA6,//13,14,15,16,17,18
		};
        /// Pointer
        typedef std::unique_ptr<CollisionDetectionColdet> Ptr;

        /// Constructor
        CollisionDetectionColdet (void);

        /// Overloaded Constructor
        CollisionDetectionColdet(std::string configFilename) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET){
            tinyxml2::XMLDocument config;
            std::string filename = configFilename;
			std::cout << filename << "\n";
			//std::string filename = "C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
			{
                std::cout << "unable to load config file.\n";
			}
			else
			{
				tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
				double query[4];
				posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
				double queryPos[4];
				posXML->QueryDoubleAttribute("x", &queryPos[0]); posXML->QueryDoubleAttribute("y", &queryPos[1]); posXML->QueryDoubleAttribute("z", &queryPos[2]);
				pose = Quaternion (query[0], query[1], query[2], query[3])*Vec3(queryPos[0], queryPos[1], queryPos[2]);

				nazwy_czesci[0]=config.FirstChildElement("Platform")->FirstChildElement("name")->GetText();
				nazwy_czesci[1]=config.FirstChildElement("Link0")->FirstChildElement("name")->GetText();
				nazwy_czesci[2]=config.FirstChildElement("Link1")->FirstChildElement("name")->GetText();
				nazwy_czesci[3]=config.FirstChildElement("Link2")->FirstChildElement("name")->GetText();

				coldet::float_type param;
				tinyxml2::XMLElement * element = config.FirstChildElement( "Platform" );
				element = element->FirstChildElement( "parameters" );
				element->QueryDoubleAttribute("length", &param); platform_length = param; element->QueryDoubleAttribute("width", &param); platform_width = param;

				element = config.FirstChildElement( "Link0" );
				element = element->FirstChildElement( "parameters" );
				element->QueryDoubleAttribute("length", &param); links_lengths[0] = param;

				element = config.FirstChildElement( "Link1" );
				element = element->FirstChildElement( "parameters" );
				element->QueryDoubleAttribute("length", &param); links_lengths[1] = param;

				element = config.FirstChildElement( "Link2" );
				element = element->FirstChildElement( "parameters" );
				element->QueryDoubleAttribute("length", &param); links_lengths[2] = param;

				element = config.FirstChildElement( "platform_orientation" );
				element->QueryDoubleAttribute("x", &param); platform_orientation[0] = param;
				element->QueryDoubleAttribute("y", &param); platform_orientation[1] = param;
				element->QueryDoubleAttribute("z", &param); platform_orientation[2] = param;
				element->QueryDoubleAttribute("alfa", &param); platform_orientation[3] = param;
				element->QueryDoubleAttribute("beta", &param); platform_orientation[4] = param;
				element->QueryDoubleAttribute("gamma", &param); platform_orientation[5] = param;

				Joint0[0]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("x")->GetText());
				Joint0[1]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("y")->GetText());
				Joint0[2]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("z")->GetText());
				Joint0[3]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("alfa")->GetText());
				Joint0[4]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("beta")->GetText());
				Joint0[5]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("gamma")->GetText());

				Joint1[0]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("x")->GetText());
				Joint1[1]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("y")->GetText());
				Joint1[2]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("z")->GetText());
				Joint1[3]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("alfa")->GetText());
				Joint1[4]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("beta")->GetText());
				Joint1[5]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("gamma")->GetText());

				Joint2[0]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("x")->GetText());
				Joint2[1]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("y")->GetText());
				Joint2[2]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("z")->GetText());
				Joint2[3]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("alfa")->GetText());
				Joint2[4]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("beta")->GetText());
				Joint2[5]=std::stof(config.FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("gamma")->GetText());

				std::cout << nazwy_czesci[0] << " length is: " << platform_length << " and width is: " << platform_width <<"\n";
				for(int i=1; i<4; i++)
				std::cout << nazwy_czesci[i] << " length is: " << links_lengths[i-1] <<"\n";

			}


			robot_model.ObjLoad("C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/Messor_II_Model/corpus.3ds");
			robot_model.ObjLoad("C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/Messor_II_Model/coxa.3ds");
			robot_model.ObjLoad("C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/Messor_II_Model/femur.3ds");
			robot_model.ObjLoad("C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/Messor_II_Model/vitulus.3ds");

			for (int i=0;i<19;i++) {
				CollisionModel3D* tmp = newCollisionModel3D();
				meshModel.push_back(tmp);
			}
			//	InitializeTerrain();
			CollisionModels();	// Init Collision Models
			//	robot_model.TerrainCollisionModels();	// Init Collision Models
			initStructures();
			for (int j=0;j<4;j++){
				cout<<"Number of "<<nazwy_czesci[j]<<" verticies is: "<<robot_model.object[j].vertices_qty<<"\n";
			}

		}
	
        /// Name of the CollisionDetectionColdet
        const std::string& getName() const;

		/// Destructor
		~CollisionDetectionColdet (void);

		/// Draw robot using openGL
		void GLDrawRobot( std::vector<coldet::float_type> config, bool * collision_table=0) const;

		/// Check collisions
		bool checkCollision( std::vector<coldet::float_type> config, bool * collision_table) const;

		//Position of the robot
		coldet::Mat34 pose;

       
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
		void Leg1(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34 * m_noga) const;
		void Leg2(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34 * m_noga) const;
		void Leg3(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34 * m_noga) const;
		void Leg4(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34 * m_noga) const;
		void Leg5(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34 * m_noga) const;
		void Leg6(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34 * m_noga) const;
		void GLLeg1(float Qn_1, float Qn_2, float Qn_3, bool * collision_table=0) const;
		void GLLeg2(float Qn_1, float Qn_2, float Qn_3, bool * collision_table=0) const;
		void GLLeg3(float Qn_1, float Qn_2, float Qn_3, bool * collision_table=0) const;
		void GLLeg4(float Qn_1, float Qn_2, float Qn_3, bool * collision_table=0) const;
		void GLLeg5(float Qn_1, float Qn_2, float Qn_3, bool * collision_table=0) const;
		void GLLeg6(float Qn_1, float Qn_2, float Qn_3, bool * collision_table=0) const;
		void copyTable(coldet::Mat34 * src, float * dest) const;
		void DrawRobot(std::vector<coldet::float_type> config) const;

		//zadane katy dla serwomechanizmow
		//float angles[18];
		///model 3DS
		std::vector<CollisionModel3D*> meshModel;
		CObjects3DS robot_model;

		std::string nazwy_czesci[4];   // [0]- Platform,  [1]- Link0,  [2]- Link1,  [3]- Link2
		//std::vector<coldet::float_type> links_lengths(size_t size=3);
		coldet::float_type links_lengths[3];  // [0] - Link0 (Coxa),  [1] - Link1 (Femur),  [2] - Link2 (Vitulus)
		coldet::float_type platform_length;
		coldet::float_type platform_width;
		//std::vector<coldet::float_type> joint1(6);
		coldet::float_type Joint0[6];
		coldet::float_type Joint1[6];
		coldet::float_type Joint2[6];
		coldet::float_type platform_orientation[6];


};

#endif // COLLISIONDETECTIONCOLDET_H_INCLUDED
