#ifndef rscurve_h
#define rscurve_h

#include "base_algorithm.h"
#include "renderbase.h"
#include "rs_math.h"
#include "rsmotion.h"
#include "coordinatesystem.h"
#include <iostream>
#include <list>
#include <iterator>
#include <algorithm>
#include <vector>
#include <set>
#include <random>

class RSCurve : public BaseAlgorithm, public Parameter
{
public:
    bool first=true;
    bool firstExpand=true;
    int lineWidth=5;
    float lineColor[3]={0.5,0.5,0.5};
    std::vector<int> start;
    std::vector<int> goal;
    float wheelbase=1.0f;
    rsmotion::algorithm::Path rsPath;
    rsmotion::CarState startPos;
    rsmotion::PointState goalPos;
    const int eps=3;
    std::vector<int> latest;
    const int collisionCircleDia=1;
    // std::vector<pathElement> solution;
    bool goalFlag=false;
    int goalCount=0;

    enum expandState
    {
        REACHED,
        ADVANCED,
        TRAPPED,
    };
    
    class pathElement
    {
    public:
        rsmotion::algorithm::Path rsPath;
        rsmotion::CarState startPos;
    };
    
    std::vector<pathElement> result; // this is solution

    void Build_tree(void);
    void Extend(std::vector<int> q);
    std::vector<int> GetNearestNeighbor(std::vector<int> q);
    std::vector<int> GetPartialPoint(std::vector<int> q, std::vector<int> qNear);
    bool CollisionCheck(pathElement pE);
    void RenderAllTree(void);
    bool IsGoalClose(void);

    RSCurve()
	{
	}
    ~RSCurve(){}

	void Initialize(void) override
    {   
        start=map->GetStartPoint();
        goal=map->GetEndPoint();

        srand(time(0));

        std::cout<<"start:"<<start[1]<<" , "<<start[0]<<std::endl;
        std::cout<<"goal:"<<goal[1]<<" , "<<goal[0]<<std::endl;

        startPos=StartPointToRSCarState(start[1], start[0], 0);
        goalPos=GoalPointToRSPointState(goal[1], goal[0], 90);

        pathElement newElem;
        newElem.startPos=startPos;
        result.push_back(newElem);
    }
	void UpdateOneStep(void) override
    {
// std::cout<<__FUNCTION__<<__LINE__<<std::endl;
// std::cin.get();
        if(true!=goalFlag)
        {
            if(true==first)
            {
                Initialize();
                first=false;
            }
            Build_tree();
        }
    }
    bool ReturnThepath(void) override
    {
        // This function has not been implemented yet.
        return false;
    }
	void Render(void) override
    {
        RenderBaseGrid();
        RenderAllTree();
    }
    bool IsTerminated(void) override 
    {
        if(latest[0]==goal[1] && latest[1]==goal[0])
        {
std::cout<<"goal= "<<goal[1]<<" , "<<goal[0]<<std::endl;
std::cout<<"latest= "<<latest[0]<<" , "<<latest[1]<<std::endl;

std::cout<<__FUNCTION__<<__LINE__<<std::endl;
std::cout<<"Reach the goal!!!"<<std::endl;
// std::cin.get();

            goalFlag=true;
            ++goalCount;
            // return true;
        }

        if(3<goalCount)
        {
std::cout<<"Reach the goal!!! ver2"<<std::endl;
// std::cin.get();
            return true;
        }
        return false;
    }

    
    /// ****** Codes below are the original algorithm in RS curve ****** ///
    rsmotion::CarState StartPointToRSCarState(const int& startX, const int& startY, const int& theta) // Should be WayPointToRSPointState
    {
        auto Position=ConvertFloatToVec3f((float)startX, 0.f, (float)startY);
        auto Orientation=ConvertFloatToQuatf(0.f, 1.0f, 0.f, (float)(theta+90)); // 90 is a coordinate dif between world coordinate and rs coordinate
    
        return {{Position, Orientation}, wheelbase};
    }
    rsmotion::PointState GoalPointToRSPointState(const int& goalX, const int& goalY, const int& theta) // Should be WayPointToRSPointState
    {
        auto Position=ConvertFloatToVec3f((float)goalX, 0.f, (float)goalY);
        auto Orientation=ConvertFloatToQuatf(0.f, 1.0f, 0.f, (float)(theta+90)); // 90 is a coordinate dif between world coordinate and rs coordinate
    
        return {Position, Orientation};
    }
    const std::vector<int> RSStateTo2DPoint(const rsmotion::CarState& carState) const
    {   
        std::vector<int> point={Round(carState.Rear.Pos[0]),Round(carState.Rear.Pos[2])};
        // waypoint.Set(Round(carState.Rear.Pos[0]),Round(carState.Rear.Pos[2]),(2*(rsmotion::math::ArcCos(carState.Rear.Orientation.Real())).ValueInDegrees()-90));

        // point.push_back(carState.Rear.Pos[0],carState.Rear.Pos[2]);

        // point.push_back(carState.Rear.Pos[0]);
        // point.push_back(carState.Rear.Pos[2]);
        return point;
    }
    // const Decision::Waypoint RSStateToWaypoint(const rsmotion::CarState& carState, const rsmotion::CarState& startCarState) const
    // {   
    //     Decision::Waypoint waypoint;
    //     waypoint.Set(Round(carState.Rear.Pos[0]),Round(carState.Rear.Pos[2])
    //     ,(2*(rsmotion::math::ArcCos(startCarState.Rear.Orientation.Real())).ValueInDegrees()-90)-(2*(rsmotion::math::ArcCos(carState.Rear.Orientation.Real())).ValueInDegrees()-90));
    
    //     return waypoint;
    // }

    std::vector<int> GetGoalPos(rsmotion::algorithm::Path path, rsmotion::CarState startPos)
    {   
        // auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, rsPath, startPos);
        auto movedCar = rsmotion::TraversePathNormalized(1.0, path, startPos);
        auto& p = movedCar.Rear.Pos;

        // Set the waypoints pos
        // int pointX = Round(p[0]) * GRIDWIDTH + GRIDWIDTH / 2 + offsetX;
        // int pointY = Round(p[2]) * GRIDHEIGHT + GRIDHEIGHT / 2 + offsetY;
        // int pointX = Round(p[0]) * GRIDWIDTH + GRIDWIDTH / 2;
        // int pointY = Round(p[2]) * GRIDHEIGHT + GRIDHEIGHT / 2;
        int pointX = Round(p[0]);
        int pointY = Round(p[2]);

        std::vector<int> pos={pointX,pointY};
        return pos;
    }

    void RenderTraversePoint(int offsetX, int offsetY, pathElement pE) const
    {
        int diameter = 3;
        double pointX = 0;
        double pointY = 0;

        float cumultiveDis=0.0;
        float cumultiveSegmentLen=0.0;
        int numPoint=500;

        // for rendering the collision area
        for(int n = 0; n < numPoint+1; ++n) {                

            // auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, rsPath, startPos);
            auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, pE.rsPath, pE.startPos);
            auto& p = movedCar.Rear.Pos;

            // Set the waypoints pos
            pointX = p[0] * GRIDWIDTH + GRIDWIDTH / 2 + offsetX;
            pointY = p[2] * GRIDHEIGHT + GRIDHEIGHT / 2 + offsetY;
            
            // for rendering the path(current ver.)
            RenderBase::DrawCircle(pointX, pointY, diameter, 0.0, 0.0, 0.0);    
            
            // if(0==n%20){ RenderBase::DrawCircleBoundary(pointX, pointY, GRIDWIDTH, 0.0, 0.0, 1.0); } // for collision circle
        }

               
        // for rendering the path(previous ver.)
        // for(int n = 0; n < numPoint+1; ++n) {                
        //     auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, pE.rsPath, pE.startPos);
        //     auto& p = movedCar.Rear.Pos;

        //     // Set the waypoints pos
        //     pointX = p[0] * GRIDWIDTH + GRIDWIDTH / 2 + offsetX;
        //     pointY = p[2] * GRIDHEIGHT + GRIDHEIGHT / 2 + offsetY;
            
        //     for (auto i = 0; i < pE.rsPath.Segments.size(); ++i)
        //     {
        //         auto &segment = pE.rsPath.Segments[i];
        //         cumultiveSegmentLen+=fabs(segment.Distance); // fabs should be replace for crossplatform dependancy

        //         if(cumultiveDis < cumultiveSegmentLen)
        //         {
        //             if(rsmotion::algorithm::SegmentDirection::Fwd==segment.Direction) // if the vehicle moves forward, the path is green. 
        //             {
        //                 RenderBase::DrawCircle(pointX, pointY, diameter, 0.0, 1.0, 0.0);
        //                 cumultiveSegmentLen=0.f;
        //                 break;
        //             }
        //             else if(rsmotion::algorithm::SegmentDirection::Bwd==segment.Direction) // if the vehicle moves backward, the path is red.
        //             {
        //                 RenderBase::DrawCircle(pointX, pointY, diameter, 1.0, 0.0, 0.0);    
        //                 cumultiveSegmentLen=0.f;
        //                 break;
        //             }
        //         }
        //     }
        //     cumultiveDis+=(1.0f / (float)numPoint)*(pE.rsPath.Length());
        // }         
    }
    
    template <typename T>
    static int Round(T x)
    {
        return static_cast<int>(x<0.0 ? x-0.5 : x+0.5);
    }
    
    rsmotion::math::Vec3f ConvertFloatToVec3f(float x, float y, float z)
    {
        rsmotion::math::Vec3f vec(x, y, z);
        return vec;
    }
    rsmotion::math::Quatf ConvertFloatToQuatf(float x, float y, float z, float a)
    {
        rsmotion::math::Quatf quatf(rsmotion::math::Vec3f{x,y,z}, rsmotion::math::Anglef::Degrees(a) );
        return quatf;
    }

    rsmotion::math::Quatf DegreeToQuatfAngle(float x, float y, float z, float a)
    {
        rsmotion::math::Quatf quatf(rsmotion::math::Vec3f{x,y,z}, rsmotion::math::Anglef::Degrees(a) );
        return quatf;
    }
    
    rsmotion::math::Quatf QuatfAngleToDegree(float x, float y, float z, float a)
    {
        rsmotion::math::Quatf quatf(rsmotion::math::Vec3f{x,y,z}, rsmotion::math::Anglef::Degrees(a) );
        return quatf;
    }
};

#endif /* rscurve_h */