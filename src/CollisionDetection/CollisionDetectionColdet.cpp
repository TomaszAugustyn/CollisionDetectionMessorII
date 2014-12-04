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

CollisionDetectionColdet::CollisionDetectionColdet(void) : CollisionDetection("Messor_II Robot Structure", TYPE_COLDET) {

//	for (int i=0;i<18;i++)
//	angles[i]=0;
//	angles[0]=45*3.14/180;
//	angles[1]=-45*3.14/180;
	robot_model.ObjLoad("resources/Messor_II_model/corpus.3ds");
	robot_model.ObjLoad("resources/Messor_II_model/coxa.3ds");
	robot_model.ObjLoad("resources/Messor_II_model/femur.3ds");
	robot_model.ObjLoad("resources/Messor_II_model/vitulus.3ds");
	
	for (int i=0;i<19;i++) {
		CollisionModel3D* tmp = newCollisionModel3D();
		meshModel.push_back(tmp);
	}
//	InitializeTerrain();
	CollisionModels();	// Init Collision Models
//	robot_model.TerrainCollisionModels();	// Init Collision Models
	initStructures();
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
	initCollisionModel(0, *meshModel[PLATFORM]); // korpus

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
	initCollisionModel(3, *meshModel[VITULUS6]); // lydka_6
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
	glColor3f(0.5,0.5,0.5);
	robot_model.Object3DS(0);
	glEndList();
}

void CollisionDetectionColdet::structCoxa(void)
{
	glNewList(GL_COXA, GL_COMPILE);
	glColor3f(0.5,0.5,0.5);
	robot_model.Object3DS(1);
	glEndList();
}

void CollisionDetectionColdet::structFemur(void)
{
	glNewList(GL_FEMUR, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
	robot_model.Object3DS(2);
	glEndList();
}

void CollisionDetectionColdet::structVitulus(void)
{
	glNewList(GL_VITULUS, GL_COMPILE);
	glColor3f(0.6,0.6,0.6);
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

void CollisionDetectionColdet::copyTable(CPunctum * src, float * dest) const{
	for (int i=0;i<4;i++){
		for (int j=0;j<4;j++){
			dest[i+4*j]=src->getElement(i+1,j+1);
		}
	}
}

void CollisionDetectionColdet::Leg3(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float biodro_3[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,biodro_3);
	meshModel[15]->setTransform (biodro_3);

	float udo_3[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, udo_3);
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2, udo_3);
	meshModel[9]->setTransform (udo_3);

	float lydka_3[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,lydka_3);
	meshModel[3]->setTransform (lydka_3);
}

void CollisionDetectionColdet::Leg4(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float biodro_4[16];
	CPunctum m_noga1, tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,biodro_4);
	meshModel[16]->setTransform (biodro_4);
								
	float udo_4[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,udo_4);
	meshModel[10]->setTransform (udo_4);

	float lydka_4[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,lydka_4);
	meshModel[4]->setTransform (lydka_4);
}

void CollisionDetectionColdet::Leg2(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float biodro_2[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,biodro_2);
	meshModel[14]->setTransform (biodro_2);

	float udo_2[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,udo_2);
	meshModel[8]->setTransform (udo_2);
										
	float lydka_2[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,lydka_2);
	meshModel[2]->setTransform (lydka_2);
}

void CollisionDetectionColdet::Leg5(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float biodro_5[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,biodro_5);
	meshModel[17]->setTransform (biodro_5);
					
	float udo_5[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,udo_5);
	meshModel[11]->setTransform (udo_5);
										
	float lydka_5[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,lydka_5);
	meshModel[5]->setTransform (lydka_5);
}

void CollisionDetectionColdet::Leg1(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float biodro_1[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,biodro_1);
	meshModel[13]->setTransform (biodro_1);
					
	float udo_1[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2,udo_1);
	meshModel[7]->setTransform (udo_1);
										
	float lydka_1[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3,lydka_1);
	meshModel[1]->setTransform (lydka_1);
}

void CollisionDetectionColdet::Leg6(float Qn_1, float Qn_2, float Qn_3, CPunctum * m_noga) const {
	float biodro_6[16];
	CPunctum m_noga1,tmp;
	m_noga1 = (*m_noga)*tmp.makeTransformMatrix("gamma", Qn_1*3.14/180);
	copyTable(&m_noga1,biodro_6);
	meshModel[18]->setTransform (biodro_6);
					
	float udo_6[16];
	CPunctum m_noga2;
	m_noga2 = m_noga1 * tmp.makeTransformMatrix("alpha", -3.14/2)*tmp.makeTransformMatrix("x", -2.15*0.254)*tmp.makeTransformMatrix("y", -0.49*0.254)*tmp.makeTransformMatrix("z", 0.92*0.254)*tmp.makeTransformMatrix("gamma", Qn_2*3.14/180);
	copyTable(&m_noga2, udo_6);
	meshModel[12]->setTransform (udo_6);
										
	float lydka_6[16];
	CPunctum m_noga3;
	m_noga3 = m_noga2 * tmp.makeTransformMatrix("x", -6.28*0.254)*tmp.makeTransformMatrix("z", -1.69*0.254)*tmp.makeTransformMatrix("gamma", Qn_3*3.14/180);
	copyTable(&m_noga3, lydka_6);
	meshModel[6]->setTransform (lydka_6);
}

void CollisionDetectionColdet::GLLeg3(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_COXA);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
//		glCallList(GL_LEG_SHEET2);
	glPopMatrix();
	
	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(0.0,0,0,1);
	glRotatef(Qn_2,0,0,1);		
	glPushMatrix();
		glCallList(GL_FEMUR);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(0.0,0,0,1);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_VITULUS);
			glTranslatef(0,0,1.57*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
//			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
//			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg4(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_COXA);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
//		glCallList(GL_LEG_SHEET2);
	glPopMatrix();
	
	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_FEMUR);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_VITULUS);
			glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
//			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
//			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg2(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_COXA);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
//		glCallList(GL_LEG_SHEET2);
	glPopMatrix();

	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_FEMUR);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_VITULUS);
			glTranslatef(0,0,1.61*0.254);	
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
//			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
//			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg5(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_COXA);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
//		glCallList(GL_LEG_SHEET2);
	glPopMatrix();
	
	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_FEMUR);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_VITULUS);
			glTranslatef(0,0,1.61*0.254);	
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
//			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
//			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg1(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_COXA);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
//		glCallList(GL_LEG_SHEET2);
	glPopMatrix();

	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_FEMUR);
		glTranslatef(0,0,-1.91*0.254);					
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_VITULUS);
			glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
//			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
//			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::GLLeg6(float Qn_1, float Qn_2, float Qn_3) const {
	glRotatef(Qn_1,0,0,1);
	glPushMatrix();
		glCallList(GL_COXA);
		glTranslatef(-2.14*0.254,0,0.49*0.254);
		glRotatef(90,1,0,0);
		glRotatef(180,0,1,0);
		glRotatef(90,0,0,1);
//		glCallList(GL_LEG_SHEET2);
	glPopMatrix();

	glRotatef(-90,1,0,0);
	glTranslatef(-2.15*0.254,-0.49*0.254,0.92*0.254);
	glRotatef(Qn_2,0,0,1);
	glPushMatrix();
		glCallList(GL_FEMUR);
		glTranslatef(0,0,-1.91*0.254);
		glTranslatef(-6.28*0.254,0,0.22*0.254);
		glRotatef(Qn_3,0,0,1);
		glPushMatrix();
			glCallList(GL_VITULUS);
			glTranslatef(0,0,1.61*0.254);
			glTranslatef(-7.90*0.254,-0.03*0.254,-0.81*0.254);
			glRotatef(90,0,1,0);
			glRotatef(-28.6,1,0,0);
//			glCallList(GL_BASE);
			glTranslatef(0,0,-0.45*0.254);
//			glCallList(GL_FOOT);
		glPopMatrix();
	glPopMatrix();
}

void CollisionDetectionColdet::DrawRobot(coldet::float_type* pos, coldet::float_type* rot, coldet::float_type * angles) const
{
	CPunctum m4,tmp;
	m4.setEye();
	m4=tmp.makeTransformMatrix("x", pos[0]*10)*tmp.makeTransformMatrix("y", pos[2]*10-0.1)*tmp.makeTransformMatrix("z", -pos[1]*10)*tmp.makeTransformMatrix("alpha", 3.14/2+rot[0])*tmp.makeTransformMatrix("beta", -rot[1])*tmp.makeTransformMatrix("gamma", 3.14-rot[2]);
	float korpus[16];
	copyTable(&m4,korpus);
	meshModel[0]->setTransform (korpus);
	
		
//===============NOGA_3=================================

	CPunctum m_noga;
	m_noga = m4*tmp.makeTransformMatrix("x", -2.56*0.254)*tmp.makeTransformMatrix("y", -6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg3(-angles[6]*180/3.14,-angles[7]*180/3.14,-angles[8]*180/3.14,&m_noga); 

//===============NOGA_4=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 2.56*0.254)*tmp.makeTransformMatrix("y", -6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg4(angles[9]*180/3.14,-angles[10]*180/3.14,-angles[11]*180/3.14,&m_noga);	

//===============NOGA_2=================================				
	m_noga = m4*tmp.makeTransformMatrix("x", -5.1*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg2(-angles[3]*180/3.14,-angles[4]*180/3.14,-angles[5]*180/3.14,&m_noga); 

//===============NOGA_5=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 5.1*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg5(angles[12]*180/3.14,-angles[13]*180/3.14,-angles[14]*180/3.14, &m_noga);	

//===============NOGA_1=================================
	m_noga = m4*tmp.makeTransformMatrix("x", -2.56*0.254)*tmp.makeTransformMatrix("y", 6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254);
	Leg1(-angles[0]*180/3.14,-angles[1]*180/3.14,-angles[2]*180/3.14,&m_noga);	

//===============NOGA_6=================================
	m_noga = m4*tmp.makeTransformMatrix("x", 2.56*0.254)*tmp.makeTransformMatrix("y", 6.06*0.254)*tmp.makeTransformMatrix("z", 3.33*0.254)*tmp.makeTransformMatrix("gamma", 3.14);
	Leg6(angles[15]*180/3.14,-angles[16]*180/3.14,-angles[17]*180/3.14, &m_noga);	
}

void CollisionDetectionColdet::GLDrawRobot(coldet::float_type *pos, coldet::float_type * rot, std::vector<coldet::float_type> config) const {

	float GLmat[16]={rot[0], rot[4], rot[8], 0, rot[1], rot[5], rot[9], 0, rot[2], rot[6], rot[10], 0, pos[0], pos[1], pos[2], 1}; //macierz do przeksztalcen

	glPushMatrix();
		glMultMatrixf(GLmat);
		glRotatef(90,1,0,0);
		glRotatef(180,0,0,1);
		glPushMatrix();
			glCallList(GL_PLATFORM);
		glPopMatrix();
		glTranslatef(0.0f,0.0f,-4.24*0.254);
		glPushMatrix();
//			glCallList(GL_PLATFORM_TOP);
		glPopMatrix();
					
//===============LEG_3=================================
		glPushMatrix();
			glTranslatef(-2.56*0.254,-6.06*0.254,3.33*0.254);
			GLLeg3(-config[6]*180/3.14,-config[7]*180/3.14,-config[8]*180/3.14); 
		glPopMatrix();

//===============LEG_4=================================
		glPushMatrix();
			glTranslatef(2.56*0.254,-6.06*0.254,3.33*0.254);
			glRotatef(180,0,0,1);
			GLLeg4(config[9]*180/3.14,-config[10]*180/3.14,-config[11]*180/3.14);	
		glPopMatrix();

//===============LEG_1=================================				
		glPushMatrix();
			glTranslatef(-5.1*0.254,0.0,3.33*0.254);
			GLLeg2(-config[3]*180/3.14,-config[4]*180/3.14,-config[5]*180/3.14); 
		glPopMatrix();

//===============LEG_5=================================
		glPushMatrix();
			glTranslatef(5.1*0.254,0,3.33*0.254);
			glRotatef(180,0,0,1);
			GLLeg5(config[12]*180/3.14,-config[13]*180/3.14,-config[14]*180/3.14);	
		glPopMatrix();

//===============LEG_1=================================
		glPushMatrix();
			glTranslatef(-2.56*0.254,6.06*0.254,3.33*0.254);
			GLLeg1(-config[0]*180/3.14,-config[1]*180/3.14,-config[2]*180/3.14);	
		glPopMatrix();

//===============LEG_6=================================
		glPushMatrix();
			glTranslatef(2.56*0.254,6.06*0.254,3.33*0.254);
			glRotatef(180,0,0,1);
			GLLeg6(config[15]*180/3.14,-config[16]*180/3.14,-config[17]*180/3.14);	
		glPopMatrix();
	glPopMatrix();
}

bool CollisionDetectionColdet::checkCollision(coldet::float_type* pos, coldet::float_type* rot, coldet::float_type * angles, bool * collision_table) const {

	DrawRobot(pos, rot, angles);
	for (int i=0;i<44;i++){
		collision_table[i]=false;
	}

	//*******KOLIZJE KOÑCZYN ROBOTA******************************************************************
	//=========KOLIZJE pierwszym, a drugim ogniwem od korpusu  ========================
	//collision_table[0-5] pierwszy czlon koliduje
	//collision_table[6-11] drugi czlon koliduje
	//collision_table[12-17] trzeci czlon koliduje
	//collision_table[18-23] trzeci czlon koliduje z terenem
	//collision_table[24] teren koliduje
	//collision_table[25] korpus koliduje
	//uproszczony sposob
	for (int i=0;i<6;i++) {
		if ((angles[i*3+1]>(24*3.14/180+1.1))){
			collision_table[i]=true;
			collision_table[i+6]=true;
		}
	}
	for (int i=0;i<6;i++) {
		if (abs(angles[i*3])>1.57){
			collision_table[i]=true;
		}
	}
	/*
	collision_table[0]=robot_model.segment_II_model_1->collision(robot_model.przegub_typu_C_1);
	if (collision_table[0]) collision_table[6]=true;
	collision_table[1]=robot_model.segment_II_model_2->collision(robot_model.przegub_typu_C_2);
	if (collision_table[1]) collision_table[7]=true;
	collision_table[2]=robot_model.segment_II_model_3->collision(robot_model.przegub_typu_C_3);
	if (collision_table[2]) collision_table[8]=true;
	collision_table[3]=robot_model.segment_II_model_4->collision(robot_model.przegub_typu_C_4);
	if (collision_table[3]) collision_table[9]=true;
	collision_table[4]=robot_model.segment_II_model_5->collision(robot_model.przegub_typu_C_5);
	if (collision_table[4]) collision_table[10]=true;
	collision_table[5]=robot_model.segment_II_model_6->collision(robot_model.przegub_typu_C_6);
	if (collision_table[5]) collision_table[11]=true;
	*/
	//=========KOLIZJE drugimi ogniwami od korpusu roznymi nogami
	if (meshModel[FEMUR1]->collision(meshModel[FEMUR2])) {
		collision_table[6]=true; collision_table[7]=true;
	}
	if (meshModel[FEMUR2]->collision(meshModel[FEMUR3])) {
		collision_table[7]=true; collision_table[8]=true;
	}
	if (meshModel[FEMUR4]->collision(meshModel[FEMUR5])) {
		collision_table[9]=true; collision_table[10]=true;
	}
	if (meshModel[FEMUR5]->collision(meshModel[FEMUR6])) {
		collision_table[10]=true; collision_table[11]=true;
	}
	//=========KOLIZJE trzecimi ogniwami od korpusu roznymi nogami
	if (meshModel[VITULUS1]->collision(meshModel[VITULUS2])) {
		collision_table[12]=true; collision_table[13]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[VITULUS3])) {
		collision_table[13]=true; collision_table[14]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[VITULUS5])) {
		collision_table[15]=true; collision_table[16]=true;
	}
	if (meshModel[VITULUS5]->collision(meshModel[VITULUS6])) {
		collision_table[16]=true; collision_table[17]=true;
	}
	//=========KOLIZJE drugimi, a trzecimi ogniwami od korpusu miedzy roznymi nogami
	if (meshModel[VITULUS1]->collision(meshModel[FEMUR2])) {
		collision_table[7]=true; collision_table[12]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[FEMUR1])) {
		collision_table[6]=true; collision_table[13]=true;
	}
	if (meshModel[VITULUS2]->collision(meshModel[FEMUR3])) {
		collision_table[8]=true; collision_table[13]=true;
	}
	if (meshModel[VITULUS3]->collision(meshModel[FEMUR2])) {
		collision_table[7]=true; collision_table[14]=true;
	}
	if (meshModel[VITULUS4]->collision(meshModel[FEMUR5])) {
		collision_table[10]=true; collision_table[15]=true;
	}
	if (meshModel[VITULUS5]->collision(meshModel[FEMUR4])) {
		collision_table[9]=true; collision_table[16]=true;
	}
	if (meshModel[VITULUS5]->collision(meshModel[FEMUR6])) {
		collision_table[11]=true; collision_table[16]=true;
	}
	if (meshModel[VITULUS6]->collision(meshModel[FEMUR5])) {
		collision_table[10]=true; collision_table[17]=true;
	}
	for (int i=0;i<44;i++){
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
