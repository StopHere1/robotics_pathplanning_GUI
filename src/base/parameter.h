#ifndef problem_parameter_h
#define problem_parameter_h

#include <stdio.h>
#include <iostream>
#include <sstream>

class Parameter
{
public:	
	Parameter(){}
	~Parameter(){}
	
	static double YsPi;

	// Problem Parameter
	static int startX;
	static int startY;
	static int goalX;
	static int goalY;

	// Grid Parameter
	static int NXGRID; // number of grids in x direction
	static int NYGRID; // number of grids in y direction 
	static int GRIDWIDTH; // number of pixel of each grid in x direction
	static int GRIDHEIGHT; // number of pixel of each grid in y direction
	static int OBSTACLEWIDTH;
};

#endif /* problem_parameter_h */