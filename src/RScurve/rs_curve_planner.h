#ifndef rs_curve_planner_h
#define rs_curve_planner_h

#include <iostream>
#include <stdio.h>
#include "rs_math.h"
#include "rsmotion.h"
#include "coordinatesystem.h"
#include "parameter.h"
#include "decision.h"
#include "wl_path.h"
#include "constraint.h"
#include "fssimplewindow.h"
#include "ysglfontdata.h"

// rs_curve_planner.h and wl_path.h is by RoboConstruction team
// rsmotion.h, rs_math.h(math.h is the original), coordinatesystem.h is made by 2019 Bas Geertsema <mail@basgeertsema.com>

// rsmotion::CarFromRSState -> converts rsState to carState(world coordinate)
// rsmotion::TraversePathNormalized -> gets points along the path. This includes rsmotion::CarFromRSState.

class RSCurvePlanner : public Parameter
{
public:
    //[latest] For generating path
    // WheelLoaderPath GeneratePath_Wconstraints(const Decision& incomingDecision, const State& incomingState, const std::vector <Constraint*>& constraints)
    WheelLoaderPath GeneratePath(const Decision& incomingDecision, const State& incomingState, const std::vector <Constraint*>& constraints)
    {        
        // Create straight segments for scooping unloading motion
        rsmotion::algorithm::Segment scoopingStrSeg;
        rsmotion::algorithm::Segment unloadingStrSeg;
        scoopingStrSeg=CreateSegment(rsmotion::algorithm::SegmentType::Straight, rsmotion::algorithm::SegmentDirection::Bwd, scoopingDriLength);
        unloadingStrSeg=CreateSegment(rsmotion::algorithm::SegmentType::Straight, rsmotion::algorithm::SegmentDirection::Fwd, unloadingDriLength);

        WheelLoaderPath wlPath;
        wlPath.waypoints.scooping=incomingDecision.scooping;
        wlPath.waypoints.unloading=incomingDecision.unloading;

        Decision::Waypoint startPoint;
        startPoint.Set(wlPath.waypoints.scooping.x + scoopingMoLength, wlPath.waypoints.scooping.y, wlPath.waypoints.scooping.theta);

        Decision::Waypoint tempSPoint;
        Decision::Waypoint tempGPoint;
        tempGPoint.Set(wlPath.waypoints.unloading.x, wlPath.waypoints.unloading.y + unloadingMoLength + unloadingDriLength, wlPath.waypoints.unloading.theta);// for now, tempGPoint.y = wlPath.waypoint.unloading.y+2 <- +2 = 1 offset from truck + 1 line segment at the last
        auto goal=wlPath.WayPointToRSPointState(tempGPoint);

        int scoopingAddLen=0; // reversing length for the wl after scooping
            
        for(;;)
        {   
            // std::cout << "scoopingStrSeg.Distance:" << scoopingStrSeg.Distance << std::endl;
            tempSPoint.Set(wlPath.waypoints.scooping.x + scoopingMoLength + scoopingDriLength + scoopingAddLen, wlPath.waypoints.scooping.y, wlPath.waypoints.scooping.theta);
            wlPath.SetStartRSCarState(tempSPoint);
            // std::cout << "tempSPoint: ";
            // tempSPoint.PrintWaypoint();
            // std::cout << "tempGPoint: ";
            // tempGPoint.PrintWaypoint();
    
            wlPath.rsPath = rsmotion::SearchShortestPath(wlPath.startPos, goal);

            // Insert the line segment
            InsertSegment(wlPath, scoopingStrSeg, startPoint); // to insert a segment as the first element
            InsertSegment(wlPath, unloadingStrSeg); // to insert a segment as the first element
            
            // For checking
            // for(;;)
            // {
        	// 	FsSwapBuffers();
            //     auto key = FsInkey();
            //     if (FSKEY_ENTER == key)
            //     {
            //         break;
            //     }
            //     wlPath.RenderTraversePoint(windowFrameWidthLeft, windowFrameWidthUpper);
            //     FsPollDevice();
            // }   

            bool check=false;
            for (auto &c : constraints) // Check path constraints
            {
                check=c->ConstraintCheck(incomingState, wlPath);
                if (true == check) // if it violates any of the constraints, the checking process ends and report violation.
                {
                    break;
                }
            }
            if(true!=check) // If the path violates the constraints
            { 
                if(rsmotion::algorithm::SegmentDirection::Bwd==scoopingStrSeg.Direction) // Initialize the distance of scooping straight length
                {
                    scoopingStrSeg.Distance=-1; 
                }
                else
                {
                    scoopingStrSeg.Distance=+1;
                }
            
                scoopingAddLen=0; // Initialize the distance of additional scooping straight length to be incremented
                break;// if the path is not violating the constraints, we can exit the loop 
            } 

            // Update scooping segment not to violate the constraint
            if(rsmotion::algorithm::SegmentDirection::Bwd==scoopingStrSeg.Direction)
            {
                scoopingStrSeg.Distance-=1;
            }
            else
            {
                scoopingStrSeg.Distance+=1;
            }
            scoopingAddLen+=1;

            if(NXGRID - incomingDecision.scooping.x < scoopingAddLen)
            {
                wlPath.rsPath.Segments.clear(); // if there is no way to avoid collision with the pile, clear Segments which means the length of the path become inf.<float>
                break; 
            }
        }
        
        std::cout << "Path:" << std::to_string(wlPath.rsPath) << std::endl;
        
        wlPath.UpdateTurningP();

        return wlPath;
    }

    rsmotion::algorithm::Segment CreateSegment(rsmotion::algorithm::SegmentType type, rsmotion::algorithm::SegmentDirection dir, int len)
    {
        rsmotion::algorithm::Segment seg=rsmotion::algorithm::Segment{type, dir};
        if(rsmotion::algorithm::SegmentDirection::Fwd==dir)
        {
            seg.Distance=len;
        }
        else if(rsmotion::algorithm::SegmentDirection::Bwd==dir)
        {
            seg.Distance=len*(-1);
        }
        return seg;
    }
    void InsertSegment(WheelLoaderPath& wlPath, rsmotion::algorithm::Segment seg, Decision::Waypoint startP) // to insert a segment as the first element
    {
        wlPath.SetStartRSCarState(startP);
        wlPath.rsPath.Segments.insert(wlPath.rsPath.Segments.begin(), seg);
    }
    void InsertSegment(WheelLoaderPath& wlPath, rsmotion::algorithm::Segment seg) // to append a segment as the last element
    {
        wlPath.rsPath.Segments.push_back(seg);
    }
};

#endif  /* rs_curve_planner_h */