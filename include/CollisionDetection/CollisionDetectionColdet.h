/** @file CollisionDetectionColdet.h
 *
 * implementation - CollisionDetectionColdet
 */
#ifndef COLLISIONDETECTIONCOLDET_H_INCLUDED
#define COLLISIONDETECTIONCOLDET_H_INCLUDED

#include "../include/CollisionDetection/CollisionDetection.h"
#include "../../3rdParty/tinyXML/tinyxml2.h"
#include <memory>
#include <iostream>
#include <array>

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
			COXA1, COXA2, COXA3, COXA4, COXA5, COXA6,//1,2,3,4,5,6 
			FEMUR1, FEMUR2, FEMUR3, FEMUR4, FEMUR5, FEMUR6,//7,8,9,10,11,12
			VITULUS1, VITULUS2, VITULUS3, VITULUS4, VITULUS5, VITULUS6, //13,14,15,16,17,18


		};
        /// Pointer
        typedef std::unique_ptr<CollisionDetectionColdet> Ptr;

        /// Constructor
        CollisionDetectionColdet (void);

        /// Overloaded Constructor with vectors initialization
		CollisionDetectionColdet(std::string configFilename) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
			//Ladowanie z pelnej sciezki - niweluje problem ze program mozna zalaczyc tylko poprzez uruchomienie pliku "Demo.exe"
			//std::string filename = "C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/" + configFilename;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
			{
                std::cout << "unable to load config file.\n";
			}
			else
			{
				jointsNo=std::stoi(config.FirstChildElement("document")->FirstChildElement("conf")->FirstChildElement("jointsNo")->GetText());
				legsNo=std::stoi(config.FirstChildElement("document")->FirstChildElement("conf")->FirstChildElement("legsNo")->GetText());

				coldet::float_type param;
				std::string param_str;
				tinyxml2::XMLElement * element;

				//rezerwacja pamieci dla konkretnej liczby elementow wektora
				nazwy_czesci.reserve(jointsNo+1);
				links_lengths.reserve(jointsNo);
				polozenie_pocz.reserve(3);
				joint0.reserve(4);
				joint1.reserve(4);
				joint2.reserve(4);
				Leg.reserve(legsNo);

				/// sczytywanie nazwy i wymiarow korpusu
				std::string parName = "Part0";
				element =(config.FirstChildElement("document")->FirstChildElement(parName.c_str()));
				nazwy_czesci[0]=element->Attribute("name");
				element->QueryDoubleAttribute("length", &param); platform_length = param;
				element->QueryDoubleAttribute("width", &param); platform_width = param;

				/// sczytywanie nazwy i wymiarow poszczegolnych czesci robota
				for(int i=1; i<jointsNo+1; i++){
					parName = "Part" + std::to_string(i);
					element =(config.FirstChildElement("document")->FirstChildElement(parName.c_str()));
					nazwy_czesci[i]=element->Attribute("name");
					element->QueryDoubleAttribute("length", &param);  links_lengths[i-1] = param;
				}
				
				/// sczytywane sa parametry poczatkowe, w ktorych rysyowana bedzie pierwsza noga
				element=config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Poczatkowe");
				element->QueryDoubleAttribute("x", &param);  polozenie_pocz[0] = param;
				element->QueryDoubleAttribute("y", &param);  polozenie_pocz[1] = param;
				element->QueryDoubleAttribute("z", &param);  polozenie_pocz[2] = param;
				//nazwa 'PolozeniePocz' nie dziala, nie wiadomo dlaczego

				/// przeksztalcenia pierwszego wezla
				element=config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0");
				element->QueryDoubleAttribute("x", &param);  joint0[0] = param;
				element->QueryDoubleAttribute("z", &param);  joint0[1] = param;
				element->QueryDoubleAttribute("alfa", &param);  joint0[2] = param;
				element->QueryDoubleAttribute("gamma", &param);  joint0[3] = param;

				/// przeksztalcenia drugiego wezla
				element=config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint1");
				element->QueryDoubleAttribute("x", &param);  joint1[0] = param;
				element->QueryDoubleAttribute("z", &param);  joint1[1] = param;
				element->QueryDoubleAttribute("alfa", &param);  joint1[2] = param;
				element->QueryDoubleAttribute("gamma", &param);  joint1[3] = param;

				/// przeksztalcenia trzeciego wezla
				element=config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint2");
				element->QueryDoubleAttribute("x", &param);  joint2[0] = param;
				element->QueryDoubleAttribute("z", &param);  joint2[1] = param;
				element->QueryDoubleAttribute("alfa", &param);  joint2[2] = param;
				element->QueryDoubleAttribute("gamma", &param);  joint2[3] = param;

				/// sczytywanie pozycji kolejnych nog robota wzgledem pozycji poczatkowej
				for(int i=0; i<legsNo; i++){

					parName = "Leg" + std::to_string(i+1);
					element=(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement(parName.c_str()));
					element->QueryDoubleAttribute("x", &param);  Leg[i][0] = param;
					element->QueryDoubleAttribute("y", &param);  Leg[i][1] = param;
					element->QueryDoubleAttribute("gamma", &param);  Leg[i][2] = param;
				}

				/*joint0[0]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("x")->GetText());
				joint0[1]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("y")->GetText());
				joint0[2]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("z")->GetText());
				joint0[3]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("alfa")->GetText());
				joint0[4]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("beta")->GetText());
				joint0[5]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint0")->FirstChildElement("gamma")->GetText());
				
				joint1[0]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("x")->GetText());
				joint1[1]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("z")->GetText());
				joint1[2]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("alfa")->GetText());
				joint1[3]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint1")->FirstChildElement("gamma")->GetText());

				joint2[0]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("x")->GetText());
				joint2[1]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("z")->GetText());
				joint2[2]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("alfa")->GetText());
				joint2[3]=std::stof(config.FirstChildElement("document")->FirstChildElement("parameters")->FirstChildElement("Joint2")->FirstChildElement("gamma")->GetText()); */

				std::cout << nazwy_czesci[0] << " length is: " << platform_length << " and width is: " << platform_width <<"\n";
				for(int i=1; i<jointsNo+1; i++)
				std::cout << nazwy_czesci[i] << " length is: " << links_lengths[i-1] <<"\n";

			}

			/// Ladowanie czesci robota, zmienne a,b,c,d przyjmuja wartosc '1' gdy czesc jest zaladowana poprawnie
			char a,b,c,d;
			a=robot_model.ObjLoad("../../resources/Messor_II_Model/corpus.3ds");
			b=robot_model.ObjLoad("../../resources/Messor_II_Model/coxa.3ds");
			c=robot_model.ObjLoad("../../resources/Messor_II_Model/femur.3ds");
			d=robot_model.ObjLoad("../../resources/Messor_II_Model/vitulus.3ds");

			/// Ladowanie z pelnej sciezki - niweluje problem ze program mozna zalaczyc tylko poprzez uruchomienie pliku "Demo.exe"
			/// robot_model.ObjLoad("C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/Messor_II_Model/corpus.3ds");


			/// tworzenie modeli kolizji w zaleznosci od ilosci nog robota (zalozenie ze kazda noga ma 3 stawy)
			for (int i=0;i<3*legsNo+1;i++) {
				CollisionModel3D* tmp = newCollisionModel3D();
				meshModel.push_back(tmp);
			}
			//	InitializeTerrain();
			CollisionModels();	// Init Collision Models
			//	robot_model.TerrainCollisionModels();	// Init Collision Models
			initStructures();
			for (int j=0;j<4;j++){
				std::cout<<"Number of "<<nazwy_czesci[j]<<" vertices is: "<<robot_model.object[j].vertices_qty<<"\n";
			}

		}
	
        /// Name of the CollisionDetectionColdet
        const std::string& getName() const;

		/// Destructor
		~CollisionDetectionColdet (void);

		/// Draw robot using openGL
		void GLDrawRobot(const coldet::Mat34& pose, const std::vector<coldet::float_type>& config, std::vector<bool>& collision_table) const;

		/// Check collisions
		bool checkCollision(const coldet::Mat34& pose, const std::vector<coldet::float_type>& config, std::vector<bool>& collision_table) const;

		/// Joints number and legs number loaded from XML file
		int jointsNo;
		int legsNo;
		
       
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

		void Leg_All(int legNo, float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga, std::array<coldet::float_type, 3> Leg) const;
		void Leg1(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const;
		void Leg2(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const;
		void Leg3(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const;
		void Leg4(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const;
		void Leg5(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const;
		void Leg6(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const;

		void GLLeg_All(int legNo, float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table, std::array<coldet::float_type, 3> Leg) const;
		void GLLeg1(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const;
		void GLLeg2(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const;
		void GLLeg3(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const;
		void GLLeg4(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const;
		void GLLeg5(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const;
		void GLLeg6(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const;

		void copyTable(coldet::Mat34& src, float * dest) const;
		void DrawRobot(const coldet::Mat34& pose, const std::vector<coldet::float_type>& config) const;
		std::vector<CollisionModel3D*> meshModel;  /// model 3DS
		CObjects3DS robot_model;

		std::vector<std::string> nazwy_czesci;   /// [0]- Platform,  [1]- Link0,  [2]- Link1,  [3]- Link2
		coldet::float_type platform_length;
		coldet::float_type platform_width;
		std::vector<coldet::float_type> links_lengths;  /// [0] - Link0 (Coxa),  [1] - Link1 (Femur),  [2] - Link2 (Vitulus)
		std::vector<coldet::float_type> polozenie_pocz;
		std::vector<coldet::float_type> joint0;
		std::vector<coldet::float_type> joint1;
		std::vector<coldet::float_type> joint2;
		std::vector< std::array<coldet::float_type, 3> > Leg;
};

#endif // COLLISIONDETECTIONCOLDET_H_INCLUDED
