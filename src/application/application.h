#ifndef APPLICATION_H_IS_NOT_INCLUDED
#define APPLICATION_H_IS_NOT_INCLUDED

#include <chrono>
#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "parameter.h"
#include "space2d.h"
#include "base_algorithm.h"
#include "dijkstra.h"
#include "astar.h"
#include "rrt.h"
#include "prm.h"
#include "rscurve.h"
#include "yspng.h"

class TextString
{
protected:
	char *str = nullptr;

public:
	TextString()
	{
	}
	TextString(const TextString &incoming)
	{
		CopyFrom(incoming);
	}
	TextString &operator=(const TextString &incoming)
	{
		CopyFrom(incoming);
		return *this;
	}
	void CopyFrom(const TextString &incoming)
	{
		if (this != &incoming)
		{
			int L = incoming.size();
			clear();
			str = new char[L + 1];
			strcpy(str, incoming.c_str());
		}
	}

	TextString(const char incoming[])
	{
		set(incoming);
	}
	TextString &operator=(const char incoming[])
	{
		set(incoming);
		return *this;
	}
	void set(const char incoming[])
	{
		if (incoming != str)
		{
			clear();
			str = new char[strlen(incoming) + 1];
			strcpy(str, incoming);
		}
	}

	~TextString()
	{
		clear();
	}
	void clear(void)
	{
		if (nullptr != str)
		{
			delete[] str;
		}
		str = nullptr;
	}

	void push_back(char c)
	{
		if (nullptr == str)
		{
			str = new char[2];
			str[0] = c;
			str[1] = 0;
		}
		else
		{
			int L = strlen(str);

			char *newStr = new char[L + 2];
			strcpy(newStr, str);
			newStr[L] = c;
			newStr[L + 1] = 0;

			delete[] str;

			str = newStr;
		}
	}

	void pop_back(void)
	{
		auto L = size();
		if (0 < L)
		{
			str[L - 1] = 0;

			char *newStr = new char[L];
			strcpy(newStr, str);

			delete[] str;

			str = newStr;
		}
	}

	const char *c_str(void) const
	{
		if (nullptr != str)
		{
			return str;
		}
		return "";
	}

	size_t size(void) const
	{
		return strlen(c_str());
	}
};

void Print(const TextString &str)
{
	printf("%s\n", str.c_str());
}

class TextInput
{
protected:
	TextString title;
	TextString str;

public:
	TextString GetString(void)
	{
		return str;
	}
	void clearStr()
	{
		str.set("");
	}
	void SetTitle(const char title[])
	{
		this->title = title;
	}
	TextString GetTitle()
	{
		return title;
	}
	void RunOneStep(int fskey, char c)
	{
		if (0 != isprint(c))
		{
			str.push_back(c);
		}
		if (FSKEY_ESC == fskey)
		{
			str.clear();
		}
		if (FSKEY_BS == fskey)
		{
			str.pop_back();
		}
	}

	void DrawUserInput(double currentValue) const
	{
		glColor3ub(0, 0, 0);
		char *text = "current User Heuristic: ";
		glRasterPos2f(850.0f - 8.0f * (float)(strlen(text) / 2), 660.0f + 6.0f);
		YsGlDrawFontBitmap8x12(text);
		glColor3ub(0, 0, 0);

		char *value = new char[10];
		snprintf(value, 8, "%lf", currentValue);
		glRasterPos2f(980.0f - 8.0f * (float)(strlen(value) / 2), 660.0f + 6.0f);
		YsGlDrawFontBitmap8x12(value);

		glColor3ub(0, 0, 0);
		glRasterPos2f(980.0f - 8.0f * (float)(strlen(title.c_str()) / 2), 680.0f + 6.0f);
		YsGlDrawFontBitmap8x12(title.c_str());
		TextString cpy;
		cpy = str;
		if (0 == time(NULL) % 2)
		{
			cpy.push_back('|');
		}
		else
		{
			cpy.push_back('_');
		}
		glRasterPos2d(980.0f - 8.0f * (float)(strlen(title.c_str()) / 2), 700.0f + 6.0f);
		YsGlDrawFontBitmap8x12(cpy.c_str());
	}
};

class buttom
{
	friend class Application;

protected:
	float x = 0;
	float y = 0;
	float width = 0;
	float height = 0;
	int state = 0;
	char *title = nullptr;

public:
	buttom(float x, float y, float width, float height, char *title);
	int getstate();
	float getx();
	float gety();
	float getwidth();
	float getheight();
	void setstate(int input);
	void draw();
	bool isinsidebuttom(int mx, int my);
};

buttom::buttom(float ix, float iy, float iwidth, float iheight, char *title)
{
	x = ix;
	y = iy;
	width = iwidth;
	height = iheight;
	state = 0;
	this->title = title;
}

bool buttom::isinsidebuttom(int mx, int my)
{
	return (mx >= x - width / 2.0f) && (mx <= x + width / 2.0f) && (my >= y - height / 2.0f) && (my <= y + height / 2.0f);
}
int buttom::getstate()
{
	return state;
}

float buttom::getx()
{
	return x;
}

float buttom::gety()
{
	return y;
}

float buttom::getheight()
{
	return height;
}

float buttom::getwidth()
{
	return width;
}

void buttom::setstate(int input)
{
	this->state = input;
}

void buttom::draw()
{
	glBegin(GL_QUADS);
	glColor3ub(170, 170, 170);
	glVertex2f(x - width / 2.0f - 10.0f, y - height / 2.0f - 10.0f);
	glVertex2f(x + width / 2.0f + 10.0f, y - height / 2.0f - 10.0f);
	glVertex2f(x + width / 2.0f + 10.0f, y + height / 2.0f + 10.0f);
	glVertex2f(x - width / 2.0f - 10.0f, y + height / 2.0f + 10.0f);
	glEnd();

	glBegin(GL_QUADS);
	if (state == 0)
	{
		glColor3ub(220, 220, 51);
	}
	else
	{
		glColor3ub(180, 180, 51);
	}
	glVertex2f(x - width / 2.0f, y - height / 2.0f);
	glVertex2f(x + width / 2.0f, y - height / 2.0f);
	glVertex2f(x + width / 2.0f, y + height / 2.0f);
	glVertex2f(x - width / 2.0f, y + height / 2.0f);
	glEnd();

	glColor3ub(0, 0, 0);
	glRasterPos2f(x - 8.0f * (float)(strlen(title) / 2), y + 6.0f);
	YsGlDrawFontBitmap8x12(title);
}

class Application
{
private:
	std::chrono::high_resolution_clock::time_point t0;
	std::chrono::high_resolution_clock::time_point t1;
	bool quit = false;
	BaseAlgorithm *algorithm = nullptr;
	int currentmapidx = 0;
	int stage = 0;
	int stateToAssign = 0;
	std::vector<Space2D *> map;
	std::vector<buttom> stage0;
	std::vector<buttom> stage1;
	std::vector<buttom> stage2;
	Parameter param;
	YsRawPngDecoder background;
	char *selectedAlg = nullptr;
	TextInput textInput;

public:
	Application()
	{
		buttom Start(640.0f, 500.0f, 150.0f, 80.0f, "START");
		stage0.push_back(Start);
		buttom Editor(640.0f, 650.0f, 150.0f, 80.0f, "MAPEDITOR");
		stage0.push_back(Editor);

		buttom dij(880.0f, 300.0f, 100.0f, 60.0f, "Dijkstra");
		stage1.push_back(dij);
		buttom ast(1080.0f, 300.0f, 100.0f, 60.0f, "Astar");
		stage1.push_back(ast);
		buttom rrt(880.0f, 400.0f, 100.0f, 60.0f, "RRT");
		stage1.push_back(rrt);
		buttom prm(1080.0f, 400.0f, 100.0f, 60.0f, "PRM");
		stage1.push_back(prm);
		buttom start(980.0f, 600.0f, 100.0f, 80.0f, "START");
		stage1.push_back(start);
		buttom back(1200.0f, 200.0f, 60.0f, 60.0f, "BACK");
		stage1.push_back(back);
		buttom rs_path(980.0f, 500.0f, 100.0f, 60.0f, "RS_PATH");
		stage1.push_back(rs_path);
		buttom changeMap(1180.0f, 600.0f, 150.0f, 80.0f, "CHANGE MAP");
		stage1.push_back(changeMap);

		buttom Exit(1080.0f, 100.0f, 60.0f, 60.0f, "Exit");
		stage2.push_back(Exit);
		buttom emp(1080.0f, 200.0f, 100.0f, 60.0f, "Empty");
		stage2.push_back(emp);
		buttom obs(1080.0f, 300.0f, 100.0f, 60.0f, "Obstacle");
		stage2.push_back(obs);
		buttom startpoint(1080.0f, 400.0f, 100.0f, 60.0f, "Start");
		stage2.push_back(startpoint);
		buttom destination(1080.0f, 500.0f, 100.0f, 60.0f, "End");
		stage2.push_back(destination);
		buttom save(1080.0f, 600.0f, 100.0f, 60.0f, "Save");
		stage2.push_back(save);
		buttom changeMap2(880.0f, 600.0f, 150.0f, 80.0f, "CHANGE MAP");
		stage2.push_back(changeMap2);

		if (YSOK != background.Decode("../src/2dgrid/bg.png"))
		{
			std::cout << "PNG load error" << std::endl;
		}
		background.Flip();
		t0 = std::chrono::high_resolution_clock::now();
		t1 = std::chrono::high_resolution_clock::now();
		textInput.SetTitle("Please Enter User Heuristic(double between 0 and 3):");
	}
	~Application() {}

	void displayAlgExplanation()
	{
		if ("Dijkstra" == selectedAlg)
		{
			char *title = "Unlike Astar, Dijkstra algorithm explores all possible paths ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title) / 2), 40.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title);
			char *title2 = "from the starting point, gradually expanding outwards until it";
			glColor3ub(0, 0, 0);
			glRasterPos2f(984.0f - 8.0f * (float)(strlen(title2) / 2), 60.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title2);
			char *title3 = "reaches the destination. It does not employ a heuristic function,";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title3) / 2), 80.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title3);
			char *title4 = "making it less efficient in terms of exploration compared to A*.";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title4) / 2), 100.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title4);
			char *title5 = "However, it guarantees the shortest path without any specific";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title5) / 2), 120.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title5);
			char *title6 = "knowledge about the goal.                                    ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title6) / 2), 140.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title6);
		}
		else if ("Astar" == selectedAlg)
		{
			char *title = "Astar is a pathfinding algorithm that efficiently finds the ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title) / 2), 40.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title);
			char *title2 = "shortest route between two points in a graph or grid by using";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title2) / 2), 60.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title2);
			char *title3 = "a heuristic to guide its search. It balances exploration and ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title3) / 2), 80.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title3);
			char *title4 = "efficiency through heuristic function. Visited grids are colored ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title4) / 2), 100.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title4);
			char *title5 = "to indicate their traversal status, with darker colors representing";
			glColor3ub(0, 0, 0);
			glRasterPos2f(1000.0f - 8.0f * (float)(strlen(title5) / 2), 120.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title5);
			char *title6 = "higher costs and lighter colors indicating lower costs.";
			glColor3ub(0, 0, 0);
			glRasterPos2f(960.0f - 8.0f * (float)(strlen(title6) / 2), 140.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title6);
		}
		else if ("RRT" == selectedAlg)
		{
			char *title = "RRT(Rapidly Exploring Random Trees) efficiently explores the";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title) / 2), 40.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title);
			char *title2 = "configuration space of a robot to find feasible paths from a";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title2) / 2), 60.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title2);
			char *title3 = "starting point to a goal. RRT randomly samples new configurations";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title3) / 2), 80.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title3);
			char *title4 = "and extends a tree-like structure towards these samples, allowing";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title4) / 2), 100.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title4);
			char *title5 = "quick exploration of complex spaces. It's useful in high-dim spaces";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title5) / 2), 120.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title5);
			char *title6 = "This path planning work with different free of freedom.     ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title6) / 2), 140.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title6);
		}
		else if ("PRM" == selectedAlg)
		{
			char *title = "PRM (Probabilistic Roadmap) is a motion planning algorithm  ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title) / 2), 40.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title);
			char *title2 = "for robots that finds the shortest path between a start and ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title2) / 2), 60.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title2);
			char *title3 = "goal configuration. It does this by sampling random configurations ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title3) / 2), 80.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title3);
			char *title4 = "in the environment, connecting them with feasible paths, and then ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title4) / 2), 100.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title4);
			char *title5 = "searching for the shortest path. It is commonly used in complex ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title5) / 2), 120.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title5);
			char *title6 = "environments with obstacles and can handle non-holonomic constraints.";
			glColor3ub(0, 0, 0);
			glRasterPos2f(1000.0f - 8.0f * (float)(strlen(title6) / 2), 140.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title6);
		}
		else if ("RSCurve" == selectedAlg)
		{
			char *title = "The Reeds-Shepp curve is a shortest-path planning algorithm ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title) / 2), 40.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title);
			char *title2 = "for nonholomonic vehicles like cars.The algorithm combines  ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title2) / 2), 60.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title2);
			char *title3 = "straight line segments and curve segments to get to the goal";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title3) / 2), 80.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title3);
			char *title4 = "Reeds, J., & Shepp, L. (1990). Optimal paths for a car that ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title4) / 2), 100.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title4);
			char *title5 = "goes both forwards and backwards. Pacific Journal of Mathematics,";
			glColor3ub(0, 0, 0);
			glRasterPos2f(998.0f - 8.0f * (float)(strlen(title5) / 2), 120.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title5);
			char *title6 = "145(2), 367-393.                                            ";
			glColor3ub(0, 0, 0);
			glRasterPos2f(980.0f - 8.0f * (float)(strlen(title6) / 2), 140.0f + 6.0f);
			YsGlDrawFontBitmap8x12(title6);
			//
		}
	}

	void drawBackgound(YsRawPngDecoder &background)
	{
		// std::cout<<background.wid<< "  " << background.hei<<std::endl;
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glRasterPos2i(0, 719);
		glDrawPixels(background.wid, background.hei, GL_RGBA, GL_UNSIGNED_BYTE, background.rgba);
		glDisable(GL_BLEND);
	}
	void setMap(Space2D *map)
	{
		this->map.push_back(map);
	}
	int getStage()
	{
		return stage;
	}
	void setStage(int incoming)
	{
		this->stage = incoming;
	}
	bool NeedQuit(void) const
	{
		return quit;
	}
	void Initialize(void) {}

	void DrawCurrentAlgorithm()
	{
		char *title = "Current Algorithm: ";
		glColor3ub(0, 0, 0);
		glRasterPos2f(880.0f - 8.0f * (float)(strlen(title) / 2), 180.0f + 6.0f);
		YsGlDrawFontBitmap8x12(title);

		glRasterPos2f(1000.0f - 8.0f * (float)(strlen(selectedAlg) / 2), 180.0f + 6.0f);
		YsGlDrawFontBitmap8x12(selectedAlg);
	}

	void DrawTimeDuration(std::chrono::high_resolution_clock::time_point t0, std::chrono::high_resolution_clock::time_point t1)
	{

		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
		char *title = "Time duration of last step: ";
		glColor3ub(0, 0, 0);
		glRasterPos2f(920.0f - 8.0f * (float)(strlen(title) / 2), 200.0f + 6.0f);
		YsGlDrawFontBitmap8x12(title);

		char *time = new char[10];
		snprintf(time, 8, "%lf", time_span.count());
		// auto time =  std::to_string(time_span.count()).c_str();
		// std::cout<<std::to_string(time_span.count()).c_str()<<std::endl;
		glRasterPos2f(1080.0f - 8.0f * (float)(strlen(time) / 2), 200.0f + 6.0f);
		YsGlDrawFontBitmap8x12(time);
	}

	void RunOneStep(void)
	{
		t0 = std::chrono::high_resolution_clock::now();
		algorithm->UpdateOneStep(); // update the search planning by one step
		t1 = std::chrono::high_resolution_clock::now();
		// quit = algorithm->IsTerminated(); // Check if the search is terminated
	}
	void inputHandler(int key, int lb, int mb, int rb, int mx, int my, int mouseEvent)
	{
		if (FSKEY_ESC == key)
		{
			quit = true;
		}
		if (stage == 0)
		{ // menu
			if (stage0[0].isinsidebuttom(mx, my))
			{
				stage0[0].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stage = 1;
				}
			}
			else
			{
				stage0[0].setstate(0);
			}

			if (stage0[1].isinsidebuttom(mx, my))
			{
				stage0[1].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stage = 2; // map editor
				}
			}
			else
			{
				stage0[1].setstate(0);
			}
		}
		if (stage == 1)
		{ // planning page
			if (stage1[0].isinsidebuttom(mx, my))
			{
				stage1[0].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// Dijkstra
					SetAlgorithm("Dijkstra", map[currentmapidx]);
				}
			}
			else
			{
				stage1[0].setstate(0);
			}
			if (stage1[1].isinsidebuttom(mx, my))
			{
				stage1[1].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// Astar
					SetAlgorithm("Astar", map[currentmapidx]);
				}
			}
			else
			{
				stage1[1].setstate(0);
			}
			if (stage1[2].isinsidebuttom(mx, my))
			{
				stage1[2].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// RRT
					SetAlgorithm("RRT", map[currentmapidx]);
				}
			}
			else
			{
				stage1[2].setstate(0);
			}
			if (stage1[3].isinsidebuttom(mx, my))
			{
				stage1[3].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// PRM
					SetAlgorithm("PRM", map[currentmapidx]);
				}
			}
			else
			{
				stage1[3].setstate(0);
			}
			if (stage1[4].isinsidebuttom(mx, my))
			{
				stage1[4].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{

					RunOneStep();
				}
			}
			else
			{
				stage1[4].setstate(0);
			}
			if (stage1[5].isinsidebuttom(mx, my))
			{
				stage1[5].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stage = 0;
				}
			}
			else
			{
				stage1[5].setstate(0);
			}
			if (stage1[6].isinsidebuttom(mx, my))
			{
				stage1[6].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// RScurve
					SetAlgorithm("RSCurve", map[currentmapidx]);
				}
			}
			else
			{
				stage1[6].setstate(0);
			}
			if (stage1[7].isinsidebuttom(mx, my))
			{
				stage1[7].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// Change map
					if (currentmapidx + 1 < map.size())
					{
						currentmapidx++;
					}
					else
					{
						currentmapidx = 0;
					}

					SetAlgorithm(selectedAlg, map[currentmapidx]);
				}
			}
			else
			{
				stage1[7].setstate(0);
			}
			if (selectedAlg == "Astar")
			{
				auto c = FsInkeyChar();
				textInput.RunOneStep(key, c);
				if (FSKEY_ENTER == key)
				{
					auto inputheuristic = textInput.GetString().c_str();
					double x = strtod(inputheuristic, NULL);
					if (0 <= x && x <= 3)
					{
						algorithm->setUserHeuristic(x);
					}
					textInput.clearStr();
				}
			}
		}
		if (2 == stage)
		{ // map editor page

			if (stage2[0].isinsidebuttom(mx, my))
			{
				stage2[0].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stage = 0;
				}
			}
			else
			{
				stage2[0].setstate(0);
			}
			if (stage2[1].isinsidebuttom(mx, my)) // set empty
			{
				stage2[1].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stateToAssign = 0;
				}
			}
			else
			{
				stage2[1].setstate(0);
			}

			if (stage2[2].isinsidebuttom(mx, my)) // set empty
			{
				stage2[2].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stateToAssign = 1;
				}
			}
			else
			{
				stage2[2].setstate(0);
			}
			if (stage2[3].isinsidebuttom(mx, my)) // set empty
			{
				stage2[3].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stateToAssign = 2;
				}
			}
			else
			{
				stage2[3].setstate(0);
			}
			if (stage2[4].isinsidebuttom(mx, my)) // set empty
			{
				stage2[4].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					stateToAssign = 3;
				}
			}
			else
			{
				stage2[4].setstate(0);
			}
			if (stage2[5].isinsidebuttom(mx, my)) // set empty
			{
				stage2[5].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					map[currentmapidx]->saveMap();
				}
			}
			else
			{
				stage2[5].setstate(0);
			}
			if (stage2[6].isinsidebuttom(mx, my)) // set empty
			{
				stage2[6].setstate(1);
				if (mouseEvent == FSMOUSEEVENT_LBUTTONUP)
				{
					// Change map
					if (currentmapidx + 1 < map.size())
					{
						currentmapidx++;
					}
					else
					{
						currentmapidx = 0;
					}

					SetAlgorithm(selectedAlg, map[currentmapidx]);
				}
			}
			else
			{
				stage2[6].setstate(0);
			}
			if (0 <= mx && 720 >= mx && 0 <= my && 720 >= my)
			{
				auto index = param.NXGRID * (my / param.GRIDHEIGHT) + mx / param.GRIDWIDTH;
				if (mouseEvent == FSMOUSEEVENT_LBUTTONDOWN || lb == 1)
				{
					map[currentmapidx]->SetStates(index, stateToAssign);
				}
			}
		}
	}

	void Draw(void)
	{
		if (stage == 0)
		{
			drawBackgound(background);
			for (buttom b : stage0)
			{
				b.draw();
			}
		}
		if (stage == 1) // path planning page
		{
			displayAlgExplanation();
			DrawCurrentAlgorithm();
			DrawTimeDuration(t0, t1);
			for (buttom b : stage1)
			{
				b.draw();
			}
			if (nullptr != this->algorithm)
			{
				this->algorithm->Render();
			}
			if (selectedAlg == "Astar")
			{
				auto value = algorithm->getUserHeuristic();
				textInput.DrawUserInput(value);
			}
		}
		if (stage == 2) // map editor page
		{
			for (buttom b : stage2)
			{
				b.draw();
			}
			if (nullptr != this->algorithm)
			{
				this->algorithm->Render2DGridDefault(0, 0);
				this->algorithm->Render2DGridLine(0, 0);
			}
		}
	}

	void SetAlgorithm(std::string input, Space2D *map)
	{
		if ("Dijkstra" == input)
		{
			algorithm = new Dijkstra;
			algorithm->setmap(map);
			selectedAlg = "Dijkstra";
		}
		else if ("Astar" == input)
		{
			algorithm = new Astar;
			algorithm->setmap(map);
			selectedAlg = "Astar";
		}
		else if ("RRT" == input)
		{
			algorithm = new RRT;
			algorithm->setmap(map);
			selectedAlg = "RRT";
		}
		else if ("PRM" == input)
		{
			algorithm = new PRM;
			algorithm->setmap(map);
			selectedAlg = "PRM";
		}
		else if ("RSCurve" == input)
		{
			algorithm = new RSCurve;
			algorithm->setmap(map);
			selectedAlg = "RSCurve";
		}
		else
		{
			algorithm = nullptr;
		}
	}

	BaseAlgorithm *Getalgorithm()
	{
		return algorithm;
	}

	std::string SelectAlgorithm(void) // need to select the algorithm from window
	{
	}
};

#endif