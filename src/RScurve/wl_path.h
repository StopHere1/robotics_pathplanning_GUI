#ifndef wl_path_h
#define wl_path_h

#include <vector>
#include "decision.h"
#include "rs_math.h"
#include "rsmotion.h"
#include "coordinatesystem.h"
#include "parameter.h"
#include "renderbase.h"
#include "util.h"

class WheelLoaderPath : public RenderBase, public Util
{
public:
    rsmotion::algorithm::Path rsPath;
    rsmotion::CarState startPos;
    Decision waypoints;

    WheelLoaderPath(){}
    ~WheelLoaderPath()
    {
        CleanUp();
    }
    void CleanUp(void){}

    void SetStartRSCarState(const Decision& decision)
    {
        startPos=WayPointToRSCarState(decision.scooping);
    }
    void SetStartRSCarState(const Decision::Waypoint& waypoint)
    {
        startPos=WayPointToRSCarState(waypoint);
    }
    rsmotion::CarState WayPointToRSCarState(const Decision::Waypoint& waypoint) // Should be WayPointToRSPointState
    {
        auto Position=ConvertFloatToVec3f(waypoint.fx, 0.f, waypoint.fy);
        auto Orientation=ConvertFloatToQuatf(0.f, 1.0f, 0.f, (float)(waypoint.theta+90)); // 90 is a coordinate dif between world coordinate and rs coordinate
    
        return {{Position, Orientation}, wheelbase};
    }
    rsmotion::PointState WayPointToRSPointState(const Decision::Waypoint& waypoint) // Should be WayPointToRSPointState
    {
        auto Position=ConvertFloatToVec3f(waypoint.fx, 0.f, waypoint.fy);
        auto Orientation=ConvertFloatToQuatf(0.f, 1.0f, 0.f, (float)(waypoint.theta+90)); // 90 is a coordinate dif between world coordinate and rs coordinate
    
        return {Position, Orientation};
    }
    const Decision::Waypoint RSStateToWaypoint(const rsmotion::CarState& carState) const
    {   
        Decision::Waypoint waypoint;
        waypoint.Set(Round(carState.Rear.Pos[0]),Round(carState.Rear.Pos[2]),(2*(rsmotion::math::ArcCos(carState.Rear.Orientation.Real())).ValueInDegrees()-90));
    
        return waypoint;
    }
    const Decision::Waypoint RSStateToWaypoint(const rsmotion::CarState& carState, const rsmotion::CarState& startCarState) const
    {   
        Decision::Waypoint waypoint;
        waypoint.Set(Round(carState.Rear.Pos[0]),Round(carState.Rear.Pos[2])
        ,(2*(rsmotion::math::ArcCos(startCarState.Rear.Orientation.Real())).ValueInDegrees()-90)-(2*(rsmotion::math::ArcCos(carState.Rear.Orientation.Real())).ValueInDegrees()-90));
    
        return waypoint;
    }
    const Decision::Waypoint RSStateToWaypoint(const rsmotion::PointState& pointState) const
    {   
        Decision::Waypoint waypoint;
        waypoint.Set(Round(pointState.Pos[0]),Round(pointState.Pos[2]),(2*(rsmotion::math::ArcCos(pointState.Orientation.Real())).ValueInDegrees()-90));
    
        return waypoint;
    }
    const Decision::Waypoint RSStateToWaypoint(const rsmotion::PointState& pointState, const rsmotion::CarState& startCarState) const
    {   
        Decision::Waypoint waypoint;
        waypoint.Set(Round(pointState.Pos[0]),Round(pointState.Pos[2])
        ,(2*(rsmotion::math::ArcCos(startCarState.Rear.Orientation.Real())).ValueInDegrees()-90)-(2*(rsmotion::math::ArcCos(pointState.Orientation.Real())).ValueInDegrees()-90));
        
        return waypoint;
    }

    void UpdateTurningP(void)
    {
        float cumultiveDis=0.0;
        float cumultiveSegmentLen=0.0;
        int numPoint=1000;

        rsmotion::algorithm::SegmentDirection previousDir=rsmotion::algorithm::SegmentDirection::Bwd;
        rsmotion::algorithm::SegmentDirection currentDir=rsmotion::algorithm::SegmentDirection::Bwd;

        // Decision::Waypoint testP; // for test printing

        for(int n = 0; n < numPoint+1; ++n) {                
            auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, rsPath, startPos);
            auto& pos = movedCar.Rear.Pos;
            auto& ori = movedCar.Rear.Orientation;

            // testP=RSStateToWaypoint(movedCar); // for test printing
            // testP.PrintWaypoint(); // for test printing

            for (auto i = 0; i < rsPath.Segments.size(); ++i)
            {
                auto &segment = rsPath.Segments[i];
                cumultiveSegmentLen+=fabs(segment.Distance); // fabs should be replace for crossplatform dependancy

                previousDir=currentDir;
                if(cumultiveDis < cumultiveSegmentLen)
                {
                    if(rsmotion::algorithm::SegmentDirection::Fwd==segment.Direction) // if the vehicle moves forward, the path is green. 
                    {
                        currentDir=rsmotion::algorithm::SegmentDirection::Fwd;
                        cumultiveSegmentLen=0.f;
                        break;
                    }
                    else if(rsmotion::algorithm::SegmentDirection::Bwd==segment.Direction) // if the vehicle moves backward, the path is red.
                    {
                        currentDir=rsmotion::algorithm::SegmentDirection::Bwd;    
                        cumultiveSegmentLen=0.f;
                        break;
                    }
                }
            }
            
            if(rsmotion::algorithm::SegmentDirection::Fwd==currentDir && rsmotion::algorithm::SegmentDirection::Bwd==previousDir) // The switching point (Bwd->Fwd) is the turning point
            {
                waypoints.turning1=RSStateToWaypoint(movedCar,startPos);
                waypoints.turning2=RSStateToWaypoint(movedCar,startPos);     
            }

            cumultiveDis+=(1.0f / (float)numPoint)*(rsPath.Length());
        }        

    }

    void RenderTraversePoint(int offsetX, int offsetY) const
    {
        int diameter = 3;
        double pointX = 0;
        double pointY = 0;

        float cumultiveDis=0.0;
        float cumultiveSegmentLen=0.0;
        int numPoint=500;

        // for rendering the collision area
        for(int n = 0; n < numPoint+1; ++n) {                

            auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, rsPath, startPos);
            auto& p = movedCar.Rear.Pos;

            // Set the waypoints pos
            pointX = p[0] * GRIDWIDTH + GRIDWIDTH / 2 + offsetX;
            pointY = p[2] * GRIDDEPTH + GRIDDEPTH / 2 + offsetY;
            
            if(0==n%20){ RenderBase::DrawCircleBoundary(pointX, pointY, GRIDWIDTH, 0.0, 0.0, 1.0); } // for test
        }

        // for rendering the path
        for(int n = 0; n < numPoint+1; ++n) {                
            auto movedCar = rsmotion::TraversePathNormalized((1.0f / (float)numPoint) * n, rsPath, startPos);
            auto& p = movedCar.Rear.Pos;

            // Set the waypoints pos
            pointX = p[0] * GRIDWIDTH + GRIDWIDTH / 2 + offsetX;
            pointY = p[2] * GRIDDEPTH + GRIDDEPTH / 2 + offsetY;
            
            for (auto i = 0; i < rsPath.Segments.size(); ++i)
            {
                auto &segment = rsPath.Segments[i];
                cumultiveSegmentLen+=fabs(segment.Distance); // fabs should be replace for crossplatform dependancy

                if(cumultiveDis < cumultiveSegmentLen)
                {
                    if(rsmotion::algorithm::SegmentDirection::Fwd==segment.Direction) // if the vehicle moves forward, the path is green. 
                    {
                        RenderBase::DrawCircle(pointX, pointY, diameter, 0.0, 1.0, 0.0);
                        cumultiveSegmentLen=0.f;
                        break;
                    }
                    else if(rsmotion::algorithm::SegmentDirection::Bwd==segment.Direction) // if the vehicle moves backward, the path is red.
                    {
                        RenderBase::DrawCircle(pointX, pointY, diameter, 1.0, 0.0, 0.0);    
                        cumultiveSegmentLen=0.f;
                        break;
                    }
                }
            }
            cumultiveDis+=(1.0f / (float)numPoint)*(rsPath.Length());
        }         
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

#endif /* wl_path_h */