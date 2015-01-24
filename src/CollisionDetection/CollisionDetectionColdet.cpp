#include "../include/CollisionDetection/CollisionDetectionColdet.h"
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
//#include <chrono>
//#include <thread>

using namespace coldet;

/// A single instance of CollisionDetectionColdet
CollisionDetectionColdet::Ptr collisionDetectionColdet;

CollisionDetectionColdet::CollisionDetectionColdet(void) : CollisionDetection("CollisionDetectionColdet", TYPE_COLDET) {

	/*char a,b,c,d,e;
	a=robot_model.ObjLoad("../../resources/Messor_II_Model/corpus.3ds");
	b=robot_model.ObjLoad("../../resources/Messor_II_Model/coxa.3ds");
	c=robot_model.ObjLoad("../../resources/Messor_II_Model/femur.3ds");
	d=robot_model.ObjLoad("../../resources/Messor_II_Model/vitulus.3ds");
	/// Ladowanie z pelnej sciezki - niweluje problem ze program mozna zalaczyc tylko poprzez uruchomienie pliku "Demo.exe"
	/// robot_model.ObjLoad("C:/Users/dom/Documents/GitHub/CollisionDetectionMessorII/resources/Messor_II_Model/corpus.3ds");

	for (int i=0; i<3*legsNo+1; i++) {
		CollisionModel3D* tmp = newCollisionModel3D();
		meshModel.push_back(tmp);
	}
	// InitializeTerrain();
	CollisionModels();	// Init Collision Models
	// robot_model.TerrainCollisionModels();	// Init Collision Models
	initStructures();
	for (int j=0;j<4;j++){
		std::cout<<robot_model.object[j].vertices_qty<<"\n";
	}*/

}

CollisionDetectionColdet::~CollisionDetectionColdet(void)
{
}

void CollisionDetectionColdet::initCollisionModel(uint_fast8_t objectNo, CollisionModel3D& model) {
	for (int j=0;j<robot_model.object[objectNo].polygons_qty;j++) {
		model.addTriangle(	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].y*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].a ].z*0.254, 
							robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].y*0.254,	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].b ].z*0.254,
							robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].x*0.254, robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].y*0.254,	robot_model.object[objectNo].vertex[ robot_model.object[objectNo].polygon[j].c ].z*0.254);
	}
	model.finalize();
}

void CollisionDetectionColdet::CollisionModels(void)
{
	initCollisionModel(0, *meshModel[0]); 

	for (int i=1; i<legsNo+1; i++){
		initCollisionModel(1, *meshModel[i]);				//init Coxa (in number according to legsNo)
		initCollisionModel(2, *meshModel[i+legsNo]);		//init Femur (in number according to legsNo)
		initCollisionModel(3, *meshModel[i+2*legsNo]);		//init Vitulus (in number according to legsNo)
	}
/*	initCollisionModel(0, *meshModel[PLATFORM]); // korpus

	initCollisionModel(1, *meshModel[COXA1]); // biodro_1
	initCollisionModel(1, *meshModel[COXA2]); // biodro_2
	initCollisionModel(1, *meshModel[COXA3]); // biodro_3
	initCollisionModel(1, *meshModel[COXA4]); // biodro_4
	initCollisionModel(1, *meshModel[COXA5]); // biodro_5
	initCollisionModel(1, *meshModel[COXA6]); // biodro_6

	initCollisionModel(2, *meshModel[FEMUR1]); // udo_1
	initCollisionModel(2, *meshModel[FEMUR2]); // udo_2
	initCollisionModel(2, *meshModel[FEMUR3]); // udo_3
	initCollisionModel(2, *meshModel[FEMUR4]); // udo_4
	initCollisionModel(2, *meshModel[FEMUR5]); // udo_5
	initCollisionModel(2, *meshModel[FEMUR6]); // udo_6

	initCollisionModel(3, *meshModel[VITULUS1]); // lydka_1
	initCollisionModel(3, *meshModel[VITULUS2]); // lydka_2
	initCollisionModel(3, *meshModel[VITULUS3]); // lydka_3
	initCollisionModel(3, *meshModel[VITULUS4]); // lydka_4
	initCollisionModel(3, *meshModel[VITULUS5]); // lydka_5
	initCollisionModel(3, *meshModel[VITULUS6]); // lydka_6 */
}

void CollisionDetectionColdet::initStructures(void)
{
	structPlatform();
	structCoxa();
	structFemur();
	structVitulus();
}

void CollisionDetectionColdet::structPlatform(void)
{
	glNewList(GL_PLATFORM, GL_COMPILE);
//	glColor3f(1.0,0.0,0.0);
	robot_model.Object3DS(0);
	glEndList();
}

void CollisionDetectionColdet::structCoxa(void)
{
	glNewList(GL_COXA, GL_COMPILE);
//	glColor3f(1.0, 0.77, 0.02);
	robot_model.Object3DS(1);
	glEndList();
}

void CollisionDetectionColdet::structFemur(void)
{
	glNewList(GL_FEMUR, GL_COMPILE);
//	glColor3f(0.02, 0.25, 1.0);
	robot_model.Object3DS(2);
	glEndList();
}

void CollisionDetectionColdet::structVitulus(void)
{
	glNewList(GL_VITULUS, GL_COMPILE);
//	glColor3f(0.0,0.92,0.1);
	robot_model.Object3DS(3);
	glEndList();
}

void CollisionDetectionColdet::drawCoordinateSystem(void)
{
	glLineWidth(3);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0.5, 0, 0);
        
		glColor3f(0, 1, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0.5, 0);
        
		glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 0.5);
    glEnd();
    glLineWidth(1);
								
    glPointSize(5);
    glBegin(GL_POINTS);
        glColor3f(1, 0, 0);
        glVertex3f(0.5, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3f(0, 0.5, 0);
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 0.5);
    glEnd();
    glPointSize(1);
	glColor3f(1, 1, 1);
}

void CollisionDetectionColdet::copyTable(coldet::Mat34& src, float * dest) const{
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++){
			dest[i+4*j]=src(i,j);
		}
	}
}

void CollisionDetectionColdet::Leg_All(int legNo, float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga, std::array<coldet::float_type, 3> Leg)const {

	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[0]*0.254, Leg[1]*0.254, 0.0*0.254);
	Eigen::Vector3d trans_joint0(joint0[0]*0.254, 0.0, joint0[1]*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd ((Leg[2]+joint0[3])*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(trans_joint0) * Eigen::AngleAxisd (joint0[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	float biodro[16];
	copyTable(m_noga1,biodro);
	meshModel[legNo]->setTransform (biodro);

	float udo[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, udo);
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::AngleAxisd (joint1[3]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2, udo);
	meshModel[legNo+legsNo]->setTransform (udo);

	float lydka[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 * Eigen::AngleAxisd (joint2[3]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (joint2[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3,lydka);
	meshModel[legNo+2*legsNo]->setTransform (lydka);
}


void CollisionDetectionColdet::Leg3(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const {

	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[0][0]*0.254, Leg[0][1]*0.254, 0.0*0.254);
	Eigen::Vector3d trans_joint0(joint0[0]*0.254, 0.0, joint0[1]*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd ((Leg[0][2]+joint0[3])*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(trans_joint0) * Eigen::AngleAxisd (joint0[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	float biodro_3[16];
	copyTable(m_noga1,biodro_3);
	meshModel[15]->setTransform (biodro_3);

	float udo_3[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, udo_3);
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::AngleAxisd (joint1[3]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2, udo_3);
	meshModel[9]->setTransform (udo_3);

	float lydka_3[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 * Eigen::AngleAxisd (joint2[3]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (joint2[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3,lydka_3);
	meshModel[3]->setTransform (lydka_3);
}

void CollisionDetectionColdet::Leg4(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const {
	float biodro_4[16];
	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[1][0]*0.254, Leg[1][1]*0.254, 0.0*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd (Leg[1][2]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga1,biodro_4);
	meshModel[16]->setTransform (biodro_4);
								
	float udo_4[16];
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2,udo_4);
	meshModel[10]->setTransform (udo_4);

	float lydka_4[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 *Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3,lydka_4);
	meshModel[4]->setTransform (lydka_4);
}

void CollisionDetectionColdet::Leg2(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const {
	float biodro_2[16];
	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[2][0]*0.254, Leg[2][1]*0.254, 0.0*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd (Leg[2][2]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga1,biodro_2);
	meshModel[14]->setTransform (biodro_2);

	float udo_2[16];
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2,udo_2);
	meshModel[8]->setTransform (udo_2);
										
	float lydka_2[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 *Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3,lydka_2);
	meshModel[2]->setTransform (lydka_2);
}

void CollisionDetectionColdet::Leg5(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const {
	float biodro_5[16];
	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[3][0]*0.254, Leg[3][1]*0.254, 0.0*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd (Leg[3][2]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga1,biodro_5);
	meshModel[17]->setTransform (biodro_5);
					
	float udo_5[16];
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2,udo_5);
	meshModel[11]->setTransform (udo_5);
										
	float lydka_5[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 *Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3,lydka_5);
	meshModel[5]->setTransform (lydka_5);
}

void CollisionDetectionColdet::Leg1(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const {
	float biodro_1[16];
	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[4][0]*0.254, Leg[4][1]*0.254, 0.0*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd (Leg[4][2]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga1,biodro_1);
	meshModel[13]->setTransform (biodro_1);
					
	float udo_1[16];
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2,udo_1);
	meshModel[7]->setTransform (udo_1);
										
	float lydka_1[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 *Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3,lydka_1);
	meshModel[1]->setTransform (lydka_1);
}

void CollisionDetectionColdet::Leg6(float Qn_1, float Qn_2, float Qn_3, coldet::Mat34& m_noga) const {
	float biodro_6[16];
	Eigen::Vector3d wektor_biodro(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	Eigen::Vector3d translacja(Leg[5][0]*0.254, Leg[5][1]*0.254, 0.0*0.254);
	coldet::Mat34 m_noga1;
	m_noga1 = m_noga * Eigen::Translation3d(wektor_biodro) * Eigen::Translation3d(translacja) * Eigen::AngleAxisd (Leg[5][2]*M_PI/180, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd (Qn_1*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga1,biodro_6);
	meshModel[18]->setTransform (biodro_6);
					
	float udo_6[16];
	Eigen::Vector3d wektor_udo(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	coldet::Mat34 m_noga2;
	m_noga2 = m_noga1 * Eigen::Translation3d(wektor_udo) * Eigen::AngleAxisd (joint1[2]*M_PI/180, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd (Qn_2*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga2, udo_6);
	meshModel[12]->setTransform (udo_6);
										
	float lydka_6[16];
	Eigen::Vector3d wektor_lydka(joint2[0]*0.254, joint2[1]*0.254, joint2[2]*0.254);
	coldet::Mat34 m_noga3;
	m_noga3 = m_noga2 *Eigen::Translation3d(wektor_lydka) * Eigen::AngleAxisd (Qn_3*M_PI/180, Eigen::Vector3d::UnitZ());
	copyTable(m_noga3, lydka_6);
	meshModel[6]->setTransform (lydka_6); 
}

void CollisionDetectionColdet::GLLeg_All(int legNo, float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table, std::array<coldet::float_type, 3> Leg) const{

	glPushMatrix();
	glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	glTranslatef(Leg[0]*0.254, Leg[1]*0.254, 0.0*0.254);
		glRotatef(Leg[2],0,0,1);
		glRotatef(joint0[3],0,0,1);
		glTranslatef(joint0[0]*0.254, 0.0*0.254, joint0[1]*0.254);
		glRotatef(joint0[2],1,0,0);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[legNo]==false)
		glColor3f(0.0, 0.75, 0.0); 	
		else
			glColor3f(0.75, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glRotatef(joint1[3],0,0,1);
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);	
	glPushMatrix();
	if(collision_table[legNo+legsNo]==false)
	glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
			glRotatef(joint2[3],0,0,1);
			glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
			glRotatef(joint2[2],1,0,0);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[legNo+2*legsNo]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();

}

void CollisionDetectionColdet::GLLeg3(float Qn_1, float Qn_2, float Qn_3, std::vector<bool>& collision_table) const {

	glPushMatrix();
	glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
	glTranslatef(Leg[0][0]*0.254, Leg[0][1]*0.254, 0.0*0.254);
		glRotatef(Leg[0][2],0,0,1);
		glRotatef(joint0[3],0,0,1);
		glTranslatef(joint0[0]*0.254, 0.0*0.254, joint0[1]*0.254);
		glRotatef(joint0[2],1,0,0);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[3]==false)
		glColor3f(0.0, 0.75, 0.0); 	
		else
			glColor3f(0.75, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glRotatef(joint1[3],0,0,1);
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);	
	glPushMatrix();
	if(collision_table[9]==false)
	glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
			glRotatef(joint2[3],0,0,1);
			glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
			glRotatef(joint2[2],1,0,0);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[15]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();

}

void CollisionDetectionColdet::GLLeg4(float Qn_1, float Qn_2, float Qn_3,  std::vector<bool>& collision_table) const {

	glPushMatrix();
		glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
		glTranslatef(Leg[1][0]*0.254, Leg[1][1]*0.254, 0.0*0.254);
		glRotatef(Leg[1][2],0,0,1);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[4]==false)
		glColor3f(0.0, 0.75, 0.0);
		else
			glColor3f(0.75, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);	
	glPushMatrix();
		if(collision_table[10]==false)
		glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
			glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[16]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();
	glPopMatrix(); 
	glPopMatrix(); 
	glPopMatrix(); 
}

void CollisionDetectionColdet::GLLeg2(float Qn_1, float Qn_2, float Qn_3,  std::vector<bool>& collision_table) const {

	glPushMatrix();
		glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
		glTranslatef(Leg[2][0]*0.254, Leg[2][1]*0.254, 0.0*0.254);
		glRotatef(Leg[2][2],0,0,1);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[2]==false)
		glColor3f(0.0, 0.75, 0.0);
		else
			glColor3f(0.75, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		if(collision_table[8]==false)
		glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
		glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[14]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();
	glPopMatrix(); 
	glPopMatrix(); 
	glPopMatrix(); 
}

void CollisionDetectionColdet::GLLeg5(float Qn_1, float Qn_2, float Qn_3,  std::vector<bool>& collision_table) const {

	glPushMatrix();
		glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
		glTranslatef(Leg[3][0]*0.254, Leg[3][1]*0.254, 0.0*0.254);
		glRotatef(Leg[3][2],0,0,1);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[5]==false)
		glColor3f(0.0, 0.75, 0.0);
		else
			glColor3f(0.75, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		if(collision_table[11]==false)
		glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
			glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[17]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg1(float Qn_1, float Qn_2, float Qn_3,  std::vector<bool>& collision_table) const {

	glPushMatrix();
		glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
		glTranslatef(Leg[4][0]*0.254, Leg[4][1]*0.254, 0.0*0.254);
		glRotatef(Leg[4][2],0,0,1);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[1]==false)
		glColor3f(0.0, 0.75, 0.0);
		else
			glColor3f(0.75, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		if(collision_table[7]==false)
		glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
		glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[13]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();

	glPopMatrix();  
	glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg6(float Qn_1, float Qn_2, float Qn_3,  std::vector<bool>& collision_table) const {

	glPushMatrix();
		glTranslatef(polozenie_pocz[0]*0.254, polozenie_pocz[1]*0.254, polozenie_pocz[2]*0.254);
		glTranslatef(Leg[5][0]*0.254, Leg[5][1]*0.254, 0.0*0.254);
		glRotatef(Leg[5][2],0,0,1);
		glRotatef(Qn_1,0,0,1);
		if(collision_table[6]==false)
		glColor3f(0.0, 0.75, 0.0);
		else
			glColor3f(0.75, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_COXA);
	glPushMatrix();
	glTranslatef(joint1[0]*0.254, 0.0*0.254, joint1[1]*0.254);
	glRotatef(joint1[2],1,0,0);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		if(collision_table[12]==false)
		glColor3f(0.0, 0.5, 0.0);
		else
			glColor3f(0.5, 0.0, 0.0);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
		glCallList(GL_FEMUR);
		glTranslatef(joint2[0]*0.254, 0.0*0.254, joint2[1]*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			if(collision_table[18]==false)
			glColor3f(0.0, 0.3, 0.0);
			else
				glColor3f(0.3, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_VITULUS);
		glPopMatrix();
	glPopMatrix();
	glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::DrawRobot (const coldet::Mat34& pose, const std::vector<coldet::float_type>& config) const
{
	coldet::Mat34 m4;
	m4 = Eigen::AngleAxisd (0, Eigen::Vector3d::UnitZ());
	m4 = m4 * pose;
	float korpus[16];
	copyTable(m4,korpus);
	meshModel[0]->setTransform (korpus);	

	for(int i=1; i<legsNo+1; i++){
		int b;
		if(i % 2 == 0)
			b=1;
		else
			b=-1;
	Leg_All(i, b*config[(i-1)*jointsNo]*180/3.14,-config[(i-1)*jointsNo +1]*180/3.14,-config[(i-1)*jointsNo +2]*180/3.14, m4, Leg[i-1]);
		}

/*//===============NOGA_3=================================

	Leg3(-config[6]*180/3.14,-config[7]*180/3.14,-config[8]*180/3.14, m4); 

//===============NOGA_4=================================

	Leg4(config[9]*180/3.14,-config[10]*180/3.14,-config[11]*180/3.14, m4);	

//===============NOGA_2=================================

	Leg2(-config[3]*180/3.14,-config[4]*180/3.14,-config[5]*180/3.14, m4); 

//===============NOGA_5=================================

	Leg5(config[12]*180/3.14,-config[13]*180/3.14,-config[14]*180/3.14, m4);	

//===============NOGA_1=================================

	Leg1(-config[0]*180/3.14,-config[1]*180/3.14,-config[2]*180/3.14, m4);	

//===============NOGA_6=================================

	Leg6(config[15]*180/3.14,-config[16]*180/3.14,-config[17]*180/3.14, m4);*/

}

void CollisionDetectionColdet::GLDrawRobot(const coldet::Mat34& pose, const std::vector<coldet::float_type>& config,  std::vector<bool>& collision_table) const {

	float GLmat[16]={pose(0,0), pose(1,0), pose(2,0), pose(3,0), pose(0,1), pose(1,1), pose(2,1), pose(3,1), pose(0,2), pose(1,2), pose(2,2), pose(3,2), pose(0,3), pose(1,3), pose(2,3), pose(3,3)}; //macierz do przeksztalcen

	glPushMatrix();
		glMultMatrixf(GLmat);
		glRotatef(-90,1,0,0);
		glRotatef(0,0,0,1);
		glPushMatrix();
		if(collision_table[0]==false)
		glColor3f(0.0, 1.0, 0.0);
		else
			glColor3f(1.0, 0.0, 0.0);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
			glCallList(GL_PLATFORM);
		glPopMatrix();
	
		for(int i=1; i<legsNo+1; i++){
			glPushMatrix();
			int a;
			if(i % 2 == 0)
				a=1;
			else
				a=-1;
			GLLeg_All(i, a*config[(i-1)*jointsNo]*180/3.14,-config[(i-1)*jointsNo +1]*180/3.14,-config[(i-1)*jointsNo +2]*180/3.14, collision_table, Leg[i-1]);
			glPopMatrix();
		}
 /*//===============LEG_3================================= Nowa noga -> 1
		glPushMatrix();
			GLLeg3(-config[6]*180/3.14,-config[7]*180/3.14,-config[8]*180/3.14, collision_table); 
		glPopMatrix();

//===============LEG_4=================================  Nowa noga -> 2
		glPushMatrix();
			GLLeg4(config[9]*180/3.14,-config[10]*180/3.14,-config[11]*180/3.14, collision_table);	
		glPopMatrix();

//===============LEG_2=================================		Nowa noga -> 3		
		glPushMatrix();
			GLLeg2(-config[3]*180/3.14,-config[4]*180/3.14,-config[5]*180/3.14, collision_table); 
		glPopMatrix(); 

//===============LEG_5================================= Nowa noga -> 4
		glPushMatrix();
			GLLeg5(config[12]*180/3.14,-config[13]*180/3.14,-config[14]*180/3.14, collision_table);	
		glPopMatrix();

//===============LEG_1================================= Nowa noga -> 5
		glPushMatrix();
			GLLeg1(-config[0]*180/3.14,-config[1]*180/3.14,-config[2]*180/3.14, collision_table);	
		glPopMatrix();

//===============LEG_6================================= Nowa noga -> 6
		glPushMatrix();
			GLLeg6(config[15]*180/3.14,-config[16]*180/3.14,-config[17]*180/3.14, collision_table);	
		glPopMatrix(); */

	glPopMatrix(); 

}

bool CollisionDetectionColdet::checkCollision(const coldet::Mat34& pose, const std::vector<coldet::float_type>& config, std::vector<bool>& collision_table) const{

	/*std::vector<std::string> nazwy_temp(19);
	nazwy_temp[0]=nazwy_czesci[0];
	for (int i=1; i<7; i++)
		nazwy_temp[i]=nazwy_czesci[1] + std::to_string(i);
	for (int i=1; i<7; i++)
		nazwy_temp[i+6]=nazwy_czesci[2] + std::to_string(i);
	for (int i=1; i<7; i++)
		nazwy_temp[i+12]=nazwy_czesci[3] + std::to_string(i); */

	DrawRobot(pose, config);
	for (int i=0; i<3*legsNo+1; i++){
		collision_table[i]=false;
	}

	//*******KOLIZJE KONCZYN ROBOTA******************************************************************
	//=========KOLIZJE pierwszym, a drugim ogniwem od korpusu  ========================
	//collision_table[0] korpus koliduje
	//collision_table[1-6] pierwszy czlon koliduje
	//collision_table[7-12] drugi czlon koliduje
	//collision_table[13-18] trzeci czlon koliduje


	//uproszczony sposob
	/*for (int i=0;i<6;i++) {
		if ((config[i*3+1]>(24*3.14/180+1.1))){
			collision_table[i]=true;
			collision_table[i+6]=true;
		}
	}
	for (int i=0;i<6;i++) {
		if (abs(config[i*3])>1.57){
			collision_table[i]=true;
		}
	} */

	//=========KOLIZJE miedzy drugimi ogniwami od korpusu roznymi nogami

	for(int j=0; j<3*legsNo+1; j++){
			for(int i=0; i<3*legsNo+1; i++){
				if(i!=j){
						if (meshModel[j]->collision(meshModel[i])){
							collision_table[j]=true; collision_table[i]=true;
						}
				}
			}
	}

	/*if (meshModel[FEMUR5]->collision(meshModel[FEMUR6])) {
		collision_table[11]=true; collision_table[12]=true;
	}
	if (meshModel[FEMUR5]->collision(meshModel[FEMUR3])) {
		collision_table[11]=true; collision_table[9]=true;
	}
	if (meshModel[FEMUR3]->collision(meshModel[FEMUR1])) {
		collision_table[9]=true; collision_table[7]=true;
	}
	if (meshModel[FEMUR1]->collision(meshModel[FEMUR2])) {
		collision_table[7]=true; collision_table[8]=true;
	}
	if (meshModel[FEMUR2]->collision(meshModel[FEMUR4])) {
		collision_table[8]=true; collision_table[10]=true;
	}
	if (meshModel[FEMUR4]->collision(meshModel[FEMUR6])) {
		collision_table[10]=true; collision_table[12]=true;
	}
	//=========KOLIZJE miedzy trzecimi ogniwami od korpusu roznymi nogami
	if (meshModel[VITULUS5]->collision(meshModel[VITULUS6])) {
		collision_table[17]=true; collision_table[18]=true;
	}
	if (meshModel[VITULUS5]->collision(meshModel[VITULUS3])) {
		collision_table[17]=true; collision_table[15]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[VITULUS1])) {
		collision_table[15]=true; collision_table[13]=true;
	}
	if (meshModel[VITULUS1]->collision(meshModel[VITULUS2])) {
		collision_table[13]=true; collision_table[14]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[VITULUS4])) {
		collision_table[14]=true; collision_table[16]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[VITULUS6])) {
		collision_table[16]=true; collision_table[18]=true;
	}
	//=========KOLIZJE miedzy drugimi, a trzecimi ogniwami od korpusu miedzy roznymi nogami
	if (meshModel[VITULUS5]->collision(meshModel[FEMUR3])) {
		collision_table[17]=true; collision_table[9]=true;
	}
	if (meshModel[VITULUS5]->collision(meshModel[FEMUR6])) {
		collision_table[17]=true; collision_table[12]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[FEMUR5])) {
		collision_table[15]=true; collision_table[11]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[FEMUR1])) {
		collision_table[15]=true; collision_table[7]=true;
	}
	if (meshModel[VITULUS1]->collision(meshModel[FEMUR3])) {
		collision_table[13]=true; collision_table[9]=true;
	}
	if (meshModel[VITULUS1]->collision(meshModel[FEMUR2])) {
		collision_table[13]=true; collision_table[8]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[FEMUR1])) {
		collision_table[14]=true; collision_table[7]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[FEMUR4])) {
		collision_table[14]=true; collision_table[10]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[FEMUR2])) {
		collision_table[16]=true; collision_table[8]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[FEMUR6])) {
		collision_table[16]=true; collision_table[12]=true;
	}
	if (meshModel[VITULUS6]->collision(meshModel[FEMUR4])) {
		collision_table[18]=true; collision_table[10]=true;
	}
	if (meshModel[VITULUS6]->collision(meshModel[FEMUR5])) {
		collision_table[18]=true; collision_table[11]=true;
	}
	//=========KOLIZJE miedzy pierwszymi, a drugimi ogniwami od korpusu miedzy roznymi nogami
	if (meshModel[FEMUR5]->collision(meshModel[COXA3])) {
		collision_table[11]=true; collision_table[3]=true;
	}
	if (meshModel[FEMUR5]->collision(meshModel[COXA6])) {
		collision_table[11]=true; collision_table[6]=true;
	}
	if (meshModel[FEMUR3]->collision(meshModel[COXA5])) {
		collision_table[9]=true; collision_table[5]=true;
	}
	if (meshModel[FEMUR3]->collision(meshModel[COXA1])) {
		collision_table[9]=true; collision_table[1]=true;
	}
	if (meshModel[FEMUR1]->collision(meshModel[COXA3])) {
		collision_table[7]=true; collision_table[3]=true;
	}
	if (meshModel[FEMUR1]->collision(meshModel[COXA2])) {
		collision_table[7]=true; collision_table[2]=true;
	}
	if (meshModel[FEMUR2]->collision(meshModel[COXA1])) {
		collision_table[8]=true; collision_table[1]=true;
	}
	if (meshModel[FEMUR2]->collision(meshModel[COXA4])) {
		collision_table[8]=true; collision_table[4]=true;
	}
	if (meshModel[FEMUR4]->collision(meshModel[COXA4])) {
		collision_table[10]=true; collision_table[4]=true;
	}
	if (meshModel[FEMUR4]->collision(meshModel[COXA6])) {
		collision_table[10]=true; collision_table[6]=true;
	}
	if (meshModel[FEMUR6]->collision(meshModel[COXA4])) {
		collision_table[12]=true; collision_table[4]=true;
	}
	if (meshModel[FEMUR6]->collision(meshModel[COXA5])) {
		collision_table[12]=true; collision_table[5]=true;
	}
	//=========KOLIZJE miedzy pierwszymi, a trzecimi ogniwami od korpusu miedzy roznymi nogami
	if (meshModel[VITULUS5]->collision(meshModel[COXA3])) {
		collision_table[17]=true; collision_table[3]=true;
	}
	if (meshModel[VITULUS5]->collision(meshModel[COXA6])) {
		collision_table[17]=true; collision_table[6]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[COXA5])) {
		collision_table[15]=true; collision_table[5]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[COXA1])) {
		collision_table[15]=true; collision_table[1]=true;
	}
	if (meshModel[VITULUS1]->collision(meshModel[COXA3])) {
		collision_table[13]=true; collision_table[3]=true;
	}
	if (meshModel[VITULUS1]->collision(meshModel[COXA2])) {
		collision_table[13]=true; collision_table[2]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[COXA1])) {
		collision_table[14]=true; collision_table[1]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[COXA4])) {
		collision_table[14]=true; collision_table[4]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[COXA2])) {
		collision_table[16]=true; collision_table[2]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[COXA6])) {
		collision_table[16]=true; collision_table[6]=true;
	}
	if (meshModel[VITULUS6]->collision(meshModel[COXA4])) {
		collision_table[18]=true; collision_table[4]=true;
	}
	if (meshModel[VITULUS6]->collision(meshModel[COXA5])) {
		collision_table[18]=true; collision_table[5]=true;
	}




	//=========KOLIZJE miedzy korpusem, a pierwszymi ogniwami od niego (Miedzy Coxa1-6, a Corpus)

	if (meshModel[COXA5]->collision(meshModel[PLATFORM])) {
		collision_table[5]=true; collision_table[0]=true;
	}
	if (meshModel[COXA3]->collision(meshModel[PLATFORM])) {
		collision_table[3]=true; collision_table[0]=true;
	}
	if (meshModel[COXA1]->collision(meshModel[PLATFORM])) {
		collision_table[1]=true; collision_table[0]=true;
	}

	if (meshModel[COXA2]->collision(meshModel[PLATFORM])) {
		collision_table[2]=true; collision_table[0]=true;
	}
	if (meshModel[COXA4]->collision(meshModel[PLATFORM])) {
		collision_table[4]=true; collision_table[0]=true;
	}
	if (meshModel[COXA6]->collision(meshModel[PLATFORM])) {
		collision_table[6]=true; collision_table[0]=true;
	}

	//=========KOLIZJE miedzy korpusem, a drugimi ogniwami od niego (Miedzy Femur1-6, a Corpus)

		if (meshModel[FEMUR5]->collision(meshModel[PLATFORM])) {
		collision_table[11]=true; collision_table[0]=true;
	}
	if (meshModel[FEMUR3]->collision(meshModel[PLATFORM])) {
		collision_table[9]=true; collision_table[0]=true;
	}
	if (meshModel[FEMUR1]->collision(meshModel[PLATFORM])) {
		collision_table[7]=true; collision_table[0]=true;
	}
	if (meshModel[FEMUR2]->collision(meshModel[PLATFORM])) {
		collision_table[8]=true; collision_table[0]=true;
	}
	if (meshModel[FEMUR4]->collision(meshModel[PLATFORM])) {
		collision_table[10]=true; collision_table[0]=true;
	}
	if (meshModel[FEMUR6]->collision(meshModel[PLATFORM])) {
		collision_table[12]=true; collision_table[0]=true;
	}

	//=========KOLIZJE miedzy korpusem, a trzecimi ogniwami od niego (Miedzy Vitulus1-6, a Corpus)

	if (meshModel[VITULUS5]->collision(meshModel[PLATFORM])) {
		collision_table[17]=true; collision_table[0]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[PLATFORM])) {
		collision_table[15]=true; collision_table[0]=true;
	}
	if (meshModel[VITULUS1]->collision(meshModel[PLATFORM])) {
		collision_table[13]=true; collision_table[0]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[PLATFORM])) {
		collision_table[14]=true; collision_table[0]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[PLATFORM])) {
		collision_table[16]=true; collision_table[0]=true;
	}
	if (meshModel[VITULUS6]->collision(meshModel[PLATFORM])) {
		collision_table[18]=true; collision_table[0]=true;
	} */


	for (int i=0;i<19;i++){
		if (collision_table[i]==true) 
			return true;
	}
	return false;
}

const std::string& CollisionDetectionColdet::getName() const {
	return name;
}


CollisionDetection* coldet::createCollisionDetectionColdet(void) {
    collisionDetectionColdet.reset(new CollisionDetectionColdet());
    return collisionDetectionColdet.get();
}

CollisionDetection* coldet::createCollisionDetectionColdet(std::string configFile) {
    collisionDetectionColdet.reset(new CollisionDetectionColdet(configFile));
    return collisionDetectionColdet.get();
}
