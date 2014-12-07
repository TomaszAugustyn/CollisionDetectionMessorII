// Main.cpp : Defines the entry point for the console application.

#include "../Defs/defs.h"
#include <thread>
#include <iostream>
#include <GL/glut.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "../include/CollisionDetection/CollisionDetectionColdet.h"
#include "../include/CollisionDetection/functions.h"

//using namespace std;
/**********************************************************
 *
 * VARIABLES DECLARATION
 *
 *********************************************************/

// The width and height of your window, change them as you like
//int screen_width=640;
//int screen_height=480;
int screen_width=1024;
int screen_height=640;

// Absolute rotation values (0-359 degrees) and rotation increments for each frame
//double rotation_x=0, rotation_x_increment=0.1;
//double rotation_y=0, rotation_y_increment=0.05;
//double rotation_z=0, rotation_z_increment=0.03;
 
double rotation_x=0, rotation_x_increment=0.0;
double rotation_y=0, rotation_y_increment=0.0;
double rotation_z=0, rotation_z_increment=0.0;
double translation_x=0;
double translation_y=0;
double translation_z=0;
// Flag for rendering as lines or filled polygons
int filling=0; //0=OFF 1=ON

//Now the object is generic, the cube has annoyed us a little bit, or not?

CollisionDetection* robot_structure;
obj_type object;


/**********************************************************
 * SUBROUTINE init()
 *
 * Used to initialize OpenGL and to setup our world
 *
 *********************************************************/

void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0); // This clear the background color to black
    glShadeModel(GL_SMOOTH); // Type of shading for the polygons
   	
    // Viewport transformation
    glViewport(0,0,screen_width,screen_height);  

    // Projection transformation
    glMatrixMode(GL_PROJECTION); // Specifies which matrix stack is the target for matrix operations 
    glLoadIdentity(); // We initialize the projection matrix as identity
    gluPerspective(45.0f,(GLfloat)screen_width/(GLfloat)screen_height,10.0f,10000.0f); // We define the "viewing volume"
   
    glEnable(GL_DEPTH_TEST); // We enable the depth test (also called z buffer)
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE); // GL_FILL-> Polygon rasterization mode (polygon filled),  GL_LINE-> (not filled) filling=0;
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_DITHER);
    
    //glEnable(GL_TEXTURE_2D); // This Enable the Texture mapping

	//Load3DS(&object, "C:/Users/dom/Desktop/3ds-Poprawione2/corpus.3ds");

}



/**********************************************************
 *
 * SUBROUTINE resize(int,int)
 *
 * This routine must be called everytime we resize our window.
 * 
 *********************************************************/

void resize (int width, int height)
{
    screen_width=width; // We obtain the new screen width values and store it
    screen_height=height; // Height value

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // We clear both the color and the depth buffer so to draw the next frame
    glViewport(0,0,screen_width,screen_height); // Viewport transformation

    glMatrixMode(GL_PROJECTION); // Projection transformation
    glLoadIdentity(); // We initialize the projection matrix as identity
    gluPerspective(45.0f,(GLfloat)screen_width/(GLfloat)screen_height,10.0f,10000.0f);

    glutPostRedisplay (); // This command redraw the scene (it calls the same routine of glutDisplayFunc)
}



/**********************************************************
 *
 * SUBROUTINE keyboard(unsigned char,int,int)
 *
 * Used to handle the keyboard input (ASCII Characters)
 * 
 *********************************************************/

void keyboard (unsigned char key, int x, int y)
{
        
    switch (key)
    {
  
        case ' ':
            rotation_x_increment=0;
            rotation_y_increment=0;
            rotation_z_increment=0;
        break;
        case 'r': case 'R':
            if (filling==0)
            {
                glPolygonMode (GL_FRONT_AND_BACK, GL_FILL); // Polygon rasterization mode (polygon filled)
                filling=1;
            }   
            else 
            {
                glPolygonMode (GL_FRONT_AND_BACK, GL_LINE); // Polygon rasterization mode (polygon outlined)
                filling=0;
            }
        break;
		case '.':
			translation_z = translation_z + 0.5;
		break;
		case ',':
			translation_z = translation_z - 0.5;
		break;
		case ';':
			translation_x = translation_x + 0.5;
		break;
		case 'k': case 'K':
			translation_x = translation_x - 0.5;
		break;
		case 'o': case 'O':
			translation_y = translation_y + 0.5;
		break;
		case 'l': case 'L':
			translation_y = translation_y - 0.5;
		break;

    }
}



/**********************************************************
 *
 * SUBROUTINE keyboard(int,int,int)
 *
 * Used to handle the keyboard input (not ASCII Characters)
 * 
 *********************************************************/

void keyboard_s (int key, int x, int y)
{  
    switch (key)
    {
        case GLUT_KEY_UP:
            rotation_x = rotation_x  +1;
        break;
        case GLUT_KEY_DOWN:
            rotation_x = rotation_x -1;
        break;
        case GLUT_KEY_LEFT:
            rotation_y  = rotation_y +1;
        break;
        case GLUT_KEY_RIGHT:
            rotation_y = rotation_y -1;
        break;
    }
}

/**********************************************************
 *
 * SUBROUTINE display()
 *
 * This is our main rendering subroutine, called each frame
 * 
 *********************************************************/

void display(void)
{
    int l_index;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // This clear the background color to dark blue
    glMatrixMode(GL_MODELVIEW); // Modeling transformation
    glLoadIdentity(); // Initialize the model matrix as identity
    
    //glTranslatef(0.0,0.0,-300); // We move the object forward (the model matrix is multiplied by the translation matrix)
	//glTranslatef(0.0,0.0, -50); // We move the object forward (the model matrix is multiplied by the translation matrix)
	glTranslatef(0,0,-18.0); // We move the object forward (the model matrix is multiplied by the translation matrix)

    if (rotation_x > 359) rotation_x = 0;
    if (rotation_y > 359) rotation_y = 0;
    if (rotation_z > 359) rotation_z = 0;

    glRotatef(rotation_x,1.0,0.0,0.0); // Rotations of the object (the model matrix is multiplied by the rotation matrices)
    glRotatef(rotation_y,0.0,1.0,0.0);
    glRotatef(rotation_z,0.0,0.0,1.0);
	glTranslatef(translation_x, 0.0, 0.0);
    glTranslatef(0.0, translation_y, 0.0);
	glTranslatef(0.0, 0.0, translation_z);

	coldet::float_type pos[3]={0,0,0};
	coldet::float_type rot[11]={1,0,0,0,0,1,0,0,0,0,1};
	std::vector<coldet::float_type> config(18,0);
	robot_structure->GLDrawRobot(pos, rot, config);
	//glutWireTeapot(10);

    /*glBegin(GL_TRIANGLES); // glBegin and glEnd delimit the vertices that define a primitive (in our case triangles)
    for (l_index=0;l_index<object.polygons_qty;l_index++)
    {
        //----------------- FIRST VERTEX -----------------
   
        // Coordinates of the first vertex
        glVertex3f( object.vertex[ object.polygon[l_index].a ].x,
                    object.vertex[ object.polygon[l_index].a ].y,
                    object.vertex[ object.polygon[l_index].a ].z); //Vertex definition

        //----------------- SECOND VERTEX -----------------
        // Coordinates of the second vertex
        glVertex3f( object.vertex[ object.polygon[l_index].b ].x,
                    object.vertex[ object.polygon[l_index].b ].y,
                    object.vertex[ object.polygon[l_index].b ].z);
        
        //----------------- THIRD VERTEX -----------------
        // Coordinates of the Third vertex
        glVertex3f( object.vertex[ object.polygon[l_index].c ].x,
                    object.vertex[ object.polygon[l_index].c ].y,
                    object.vertex[ object.polygon[l_index].c ].z);
    }
    glEnd();*/

    glFlush(); // This force the execution of OpenGL commands
    glutSwapBuffers(); // In double buffered mode we invert the positions of the visible buffer and the writing buffer
}



/**********************************************************
 *
 * The main routine
 * 
 *********************************************************/

int main(int argc, char **argv)
{
	//robot_structure = createCollisionDetectionColdet();
	
    // We use the GLUT utility to initialize the window, to handle the input and to interact with the windows system
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(screen_width,screen_height);
    glutInitWindowPosition(400,200);
    glutCreateWindow("Model robota Messor II");    
	robot_structure = createCollisionDetectionColdet();
    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutReshapeFunc (resize);
    glutKeyboardFunc (keyboard);
    glutSpecialFunc (keyboard_s);
    init();
    glutMainLoop();

    return(0);    
}


/*int main()
{
	//char Load3DS (obj_type_ptr p_object, const char *p_filename);
	//obj_type_ptr p_object;
	obj_type p_object;
	char a;
	//const char *p_filename;
	//p_filename="C:/Users/dom/Desktop/3ds-nowe2/coxa2.3ds";
	//Load3DS(&p_object, p_filename);
	a=Load3DS(&p_object, "C:/Users/dom/Desktop/3ds-nowe2/coxa2.3ds");
	//p_object.vertices_qty;
	//Load3DS(&p_object, "coxa2.3ds");
	cout<<int(a)<<"\n";
	cout<<p_object.vertices_qty<<"\n";
	system("pause");
	return 0;
}*/

