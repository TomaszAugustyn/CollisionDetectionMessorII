// Main.cpp : Defines the entry point for the console application.
//
#include "../Defs/defs.h"
#include <iostream>

#include <windows.h>
#include <GL/GLU.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "../include/CollisionDetection/3dsloader.h"
#include "../include/CollisionDetection/types.h"

//using namespace std;


int main()
{
	//char Load3DS (obj_type_ptr p_object, const char *p_filename);
	obj_type_ptr p_object;
	//obj_type p_object;
	//const char *p_filename;
	//p_filename="C:/Users/dom/Desktop/3ds-nowe/corpus.3ds";
	//Load3DS(&p_object, p_filename);
	Load3DS(p_object, "C:/Users/dom/Desktop/3ds-nowe/corpus.3ds");
	return 0;
}

