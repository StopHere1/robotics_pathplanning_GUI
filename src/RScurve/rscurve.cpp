#include <time.h>
#include <cmath>
#include "rscurve.h"

void RSCurve::Build_tree(void)
{
    if(true==IsGoalClose())
    {
std::cout<<__FUNCTION__<<__LINE__<<std::endl;
std::cout<<"the goal is close to the tree"<<std::endl;
        Extend(goal);
    }
    else
    {
        std::vector<int> qRand;
        qRand.push_back(rand()%(NXGRID)); // x coordinate of random sampling point
        qRand.push_back(rand()%(NYGRID)); // y coordinate of random sampling point

        if(true==map->IsInRange(qRand[0],qRand[1]))
        {
            std::cout<<"qRand= "<<qRand[0]<<" , "<<qRand[1]<<std::endl;
            Extend(qRand);
        }
    }
}

void RSCurve::Extend(std::vector<int> q)
{
    std::vector<int> qNear=GetNearestNeighbor(q);
    std::vector<int> qPar;
    if(true!=firstExpand)
    {
        if(fabs(latest[0]-goal[0])<=eps && fabs(latest[1]-goal[1])<=eps)
        {
            std::vector<int> invq={q[1],q[0]};
            qPar=invq;
        }
        else
        {
            qPar=GetPartialPoint(q,qNear);
        }
    }
    else
    {
        qPar=GetPartialPoint(q,qNear);   
        firstExpand=false; 
    }
    rsmotion::CarState startP=StartPointToRSCarState(qNear[0], qNear[1], 0);
    rsmotion::PointState goalP=GoalPointToRSPointState(qPar[0], qPar[1], 90);

    rsmotion::algorithm::Path newPath=rsmotion::SearchShortestPath(startP, goalP);

    std::cout<<std::to_string(newPath)<<std::endl;

    pathElement newElem;
    newElem.rsPath=newPath;
    newElem.startPos=startP;

    std::vector<int> tempGoal=GetGoalPos(newPath, startP);
    std::cout<<"tempGoal= "<<tempGoal[0]<<" , "<<tempGoal[1]<<std::endl;

    // if(true==CollisionCheck(newElem) && true==map->IsInRange(tempGoal[0],tempGoal[1]))
    if(true==CollisionCheck(newElem) || true!=map->IsInRange(tempGoal[0],tempGoal[1]))
    {
        return;
    }
    else
    {
        std::cout<<__FUNCTION__<<__LINE__<<std::endl;
        result.push_back(newElem);
    }
    // update the latest point(std::vector<int>)
    latest=GetGoalPos(newPath, startP);

    std::cout<<__FUNCTION__<<__LINE__<<std::endl;
    std::cout<<"goal= "<<goal[1]<<" , "<<goal[0]<<std::endl;
    std::cout<<"latest= "<<latest[0]<<" , "<<latest[1]<<std::endl;
}

std::vector<int> RSCurve::GetNearestNeighbor(std::vector<int> q)
{
    std::vector<int> qNear={0,0};
    double dist=100000.0;
    std::vector<int> qCand;
    for(auto &s: result)
    {
        if(true==firstExpand)
        {
            std::cout<<std::to_string(s.startPos)<<std::endl;
            qCand=RSStateTo2DPoint(s.startPos);
        }
        else
        {
            qCand=GetGoalPos(s.rsPath, s.startPos);
        }

        double xDiff=q[0]-qCand[0];
        double yDiff=q[1]-qCand[1];
        double tempDist=sqrt(xDiff*xDiff+yDiff*yDiff);

        if(dist>tempDist)
        {
            dist=tempDist;
            qNear[0]=qCand[0];
            qNear[1]=qCand[1];
        }
    }

    return qNear;
}

std::vector<int> RSCurve::GetPartialPoint(std::vector<int> q, std::vector<int> qNear)
{
    double xDiff=q[0]-qNear[0];
    double yDiff=q[1]-qNear[1];
    double tempDist=sqrt(xDiff*xDiff+yDiff*yDiff);
    
    double xExp=0;
    double yExp=0;
    
    if(0!=tempDist)
    {
        xExp=xDiff*(eps/tempDist);
        yExp=yDiff*(eps/tempDist);
    }
    else
    {
        xExp=0.0;
        yExp=0.0;
    }

std::cout<<"xExp= "<<xExp<<" , "<<"yExp= "<<yExp<<std::endl;

    std::vector<int> qPar={Round(qNear[0]+xExp), Round(qNear[1]+yExp)};

    return qPar;
}

// pathElement
bool RSCurve::CollisionCheck(pathElement pE)
{
    int numPoint=1000;

    for(int n = 0; n < numPoint+1; ++n)
    {                
        auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, pE.rsPath, pE.startPos);
        auto& pos = movedCar.Rear.Pos;
        auto& ori = movedCar.Rear.Orientation;

        float centerX=pos[0];
        float centerY=pos[2];

        // std::cout<<"centerX= "<<centerX<<" , "<<"centerY= "<<centerY<<"  GridState: "<<map->GetGridState(Round(centerY), Round(centerX))<<std::endl;
        
        for (int i = 0; i < 64; ++i)
        {
            double ang = YsPi * (double)i / 32.0;
            double s = sin(ang);
            double c = cos(ang);
            float x = centerX + collisionCircleDia / 2 * c;
            float y = centerY + collisionCircleDia / 2 * s;

            auto gridX=Round(x);
            auto gridY=Round(y);
            int gridZ=0;

            // std::cout<<"gridX= "<<gridX<<" , "<<"gridY= "<<gridY<<"  GridState: "<<map->GetGridState(gridX, gridY)<<std::endl;

            // if(1==map->GetGridState(gridX, gridY) && true==map->IsInRange(gridX, gridY))
            if(1==map->GetGridState(gridY, gridX) && true==map->IsInRange(gridX, gridY))
            {
                std::cout<<"Collision!"<<std::endl;
                // std::cout<<"centerX= "<<centerX<<" , "<<"centerY= "<<centerY<<std::endl;
                // std::cout<<"gridX= "<<gridX<<" , "<<"gridY= "<<gridY<<"  GridState: "<<map->GetGridState(gridX, gridY)<<std::endl;
                // std::cout<<"X= "<<gridY<<" , "<<"Y= "<<gridY<<"  GridState: "<<map->GetGridState(gridY, gridX)<<std::endl;
                // std::cin.get();
                return true;
            }
        }
    }        
    return false;
}

void RSCurve::RenderAllTree(void)
{
    for (auto p : result)
    {
        RenderTraversePoint(0,0,p);
    }
}

bool RSCurve::IsGoalClose(void)
{
    if(true!=firstExpand)
    {
        if(fabs(latest[0]-goal[1])<=eps && fabs(latest[1]-goal[0])<=eps)
        {
            return true;
        }
    }
    return false;
}