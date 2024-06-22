#include <iostream>
#include <stdio.h>
#include "space2d.h"

void Space2D::Initialize(int wid, int hei)
{
    Create(wid,hei);
    // SetObstacle();
}
// void Space2D::SetObstacle(void)
// {   
//     for (int y = 0; y < GetHeight(); ++y)
//     {
//         for (int x = 0; x < GetWidth(); ++x)
//         {
//             // for now, an obstacle is set manually
//             if(27<x && x<29 && 5<y && y<35)
//             {
//                 Get(x, y).obstacle = true;
//             }
//             else
//             {
//                 Get(x, y).obstacle = false;
//             }
//         }
//     }
// }