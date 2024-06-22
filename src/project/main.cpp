#include <iostream>
#include <string>
#include <vector>
#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "space2d.h"
#include "base_algorithm.h"
#include "dijkstra.h"
#include "application.h"

int main(void)
{
	enum
	{
		
	};

	FsOpenWindow(0,0,1280,720,1);
	Parameter para;
	Application app;
	
	// __Get user input here__ //
	//std::string algorithm=app.SelectAlgorithm();
	Space2D map,map2,map3,map4;
	map.Initialize(para.NXGRID,para.NYGRID);
	map.LoadMap("../src/2dgrid/map2.txt");
	app.setMap(&map);
	map2.Initialize(para.NXGRID,para.NYGRID);
	map2.LoadMap("../src/2dgrid/map.txt");
	app.setMap(&map2);
	map3.Initialize(para.NXGRID,para.NYGRID);
	map3.LoadMap("../src/2dgrid/map3.txt");
	app.setMap(&map3);
	map4.Initialize(para.NXGRID,para.NYGRID);
	map4.LoadMap("../src/2dgrid/map4.txt");
	app.setMap(&map4);
	std::string alg="Dijkstra";	// for now set Dijkstra
	app.SetAlgorithm(alg,&map);

	// run algorithm oune step
	while(true!=app.NeedQuit())
	{
		FsPollDevice();
		auto key = FsInkey();
		int lb, mb, rb, mx, my, mouseEvent;
        mouseEvent = FsGetMouseEvent(lb, mb, rb, mx, my);
		app.inputHandler(key, lb,mb,rb,mx,my,mouseEvent);
		// app.RunOneStep();
	
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		app.Draw();
	
		FsSwapBuffers();
	}


	return 0;
}
