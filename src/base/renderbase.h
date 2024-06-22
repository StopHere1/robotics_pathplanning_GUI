#ifndef renderbase_h
#define renderbase_h

#include <iostream>
#include "ysglfontdata.h"
#include "parameter.h"

class RenderBase : public Parameter
{
public:
    void DrawSquare(int centerX, int centerY, int width, int height) const;
    void DrawSquare(int centerX, int centerY, int width, int height, float red, float green, float blue) const;
    void DrawSquare(int centerX, int centerY, int width, int height, float red, float green, float blue, float alpha) const;
    void DrawSquareBoundary(int centerX, int centerY, int width, int height, float red, float green, float blue) const;

    void DrawLine(int centerX1, int centerY1, int centerX2, int centerY2, int width, float red, float green, float blue) const;
    void DrawCircle(int centerX, int centerY, int diameter, float red, float green, float blue) const;
    void DrawCircleBoundary(int centerX, int centerY, int diameter, float red, float green, float blue) const;

    void DrawCubeBody(double x0, double y0, double z0, double x1, double y1, double z1, float red, float green, float blue) const;
    void DrawCubeBoundary(double x0, double y0, double z0, double x1, double y1, double z1, float red, float green, float blue) const;

    void DrawFontWithTheSize(std::string fontSize, const char* c) const;
    void DrawFontWithTheSize(std::string fontSize, double value) const;
    void DrawFontWithTheSize(std::string fontSize, const char* c, float red, float green, float blue) const;
    void DrawFontWithTheSize(std::string fontSize, double value, float red, float green, float blue) const;
};


#endif /* renderbase_h */
