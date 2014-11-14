// Main.cpp : Defines the entry point for the console application.
//
#include "../Defs/defs.h"
#include <iostream>

//#include <GL/GLU.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "../include/CollisionDetection/3dsloader.h"
#include "../include/CollisionDetection/types.h"

using namespace std;



int main()
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
}

