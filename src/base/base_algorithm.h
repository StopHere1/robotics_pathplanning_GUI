#ifndef base_algorithm_h
#define base_algorithm_h

#include "fssimplewindow.h"
#include "ysglfontdata.h"
#include "parameter.h"
#include "renderbase.h"
#include "space2d.h"
#include <vector>

class BaseAlgorithm : public RenderBase
{
public:
    BaseAlgorithm(){}
    ~BaseAlgorithm(){}

    Space2D *map = nullptr; // This should be installed in grid class

	std::vector<std::vector<int>> *path; // Outcome, the result ([0,0],[0,1]....[50,50]<-goal)

	virtual void Initialize(void)=0;
	virtual void UpdateOneStep(void)=0;
	virtual void Render(void)=0;
    virtual bool IsTerminated(void)=0;
    virtual bool ReturnThepath(void)=0;

	virtual void setUserHeuristic(double userInput){
		return;
	}
	
	virtual double getUserHeuristic()
	{
		return 0.0;
	}

    void CreateMap(void)
    {
        map->Initialize(NXGRID, NYGRID);
    }
    
	void  setmap(Space2D *map){
		this->map = map;
	}

    void RenderBaseGrid(void)
    {
        Render2DGridDefault(0, 0);
        Render2DGridLine(0, 0);
    }

    void Render2DGridLine(int offsetX, int offsetY)
	{
		glColor3ub(0, 0, 0);
		glBegin(GL_LINES); // draw vertical lines
		for (int i = 0; i <= NXGRID; ++i)
		{
			glVertex2i(i * GRIDWIDTH + offsetX, 0 + offsetY);
			glVertex2i(i * GRIDWIDTH + offsetX, NYGRID * GRIDHEIGHT + offsetY);
		}
		glEnd();
		glBegin(GL_LINES); // draw horizontal lines
		for (int i = 0; i <= NYGRID; ++i)
		{
			glVertex2i(0 + offsetX, i * GRIDHEIGHT + offsetY);
			glVertex2i(NXGRID * GRIDWIDTH + offsetX, i * GRIDHEIGHT + offsetY);
		}
		glEnd();
	}

    void Render2DGridDefault(int offsetX, int offsetY)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBegin(GL_QUADS);
		for (int y = 0; y < NYGRID; ++y)
		{
			for (int x = 0; x < NXGRID; ++x)
			{
				if (2 == map->Get(x,y).state)
				{
					glColor3f(1.0f, 0.5f, 0.0f); // start grid color
				}
				else if ( 3 == map->Get(x,y).state)
				{
					glColor3f(0.0f, 0.5f, 1.0f); // goal grid color
				}
                else if (1 == map->Get(x,y).state)
                {
                	glColor3f(1.0f, 1.0f, 1.0f); // obstacle grid color    
                }
				else
				{	
                	glColor3f(1.0f, 1.0f, 1.0f); // other grid color    
                }
				int cx = x * GRIDWIDTH + GRIDWIDTH / 2;
				int cy = y * GRIDHEIGHT + GRIDHEIGHT / 2;
				RenderBase::DrawSquare(cx + offsetX, cy + offsetY, GRIDWIDTH, GRIDHEIGHT);
				if(1==map->Get(x,y).state){
					glColor3f(0.0f, 0.0f, 0.0f); 
					int cx = x * GRIDWIDTH + GRIDWIDTH / 2;
					int cy = y * GRIDHEIGHT + GRIDHEIGHT / 2;
					RenderBase::DrawSquare(cx + offsetX, cy + offsetY, OBSTACLEWIDTH, OBSTACLEWIDTH);
					if(1==map->Get(x-1,y).state)
					{
						RenderBase::DrawSquare(cx + offsetX - GRIDWIDTH/4, cy + offsetY, GRIDWIDTH/2, OBSTACLEWIDTH);
					}
					if(1==map->Get(x+1,y).state)
					{
						RenderBase::DrawSquare(cx + offsetX + GRIDWIDTH/4, cy + offsetY, GRIDWIDTH/2, OBSTACLEWIDTH);
					}
					if(1==map->Get(x,y-1).state)
					{
						RenderBase::DrawSquare(cx + offsetX, cy + offsetY - GRIDHEIGHT/4, OBSTACLEWIDTH, GRIDHEIGHT/2 );
					}
					if(1==map->Get(x,y+1).state)
					{
						RenderBase::DrawSquare(cx + offsetX, cy + offsetY + GRIDHEIGHT/4, OBSTACLEWIDTH,  GRIDHEIGHT/2);
					}
					if(1==map->Get(x-1,y).state)
					{

					}
					
				}
			}
		}
		glEnd();
		glDisable(GL_BLEND);
	}
};

#endif /* base_algorithm_h */