#include "parameter.h"

double Parameter::YsPi = 3.1415927;

// Problem Parameter
int Parameter::startX = 5;
int Parameter::startY = 24-5;
int Parameter::goalX = 24-5; 
int Parameter::goalY = 5;


// Grid Parameter
int Parameter::NXGRID = 30; // number of grids in x direction
int Parameter::NYGRID = 30; // number of grids in y direction 
int Parameter::GRIDWIDTH = 24; // number of pixel of each grid in x direction
int Parameter::GRIDHEIGHT = 24; // number of pixel of each grid in y direction
int Parameter::OBSTACLEWIDTH = 10;