#include <sstream>
#include <iomanip>
#include "renderbase.h"
#include "fssimplewindow.h"
#include "ysglfontdata.h"

void RenderBase::DrawSquare(int centerX, int centerY, int width, int height) const
{
    glBegin(GL_QUADS);
    glVertex2i(centerX - width / 2, centerY + height / 2);
    glVertex2i(centerX - width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY + height / 2);
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}
void RenderBase::DrawSquare(int centerX, int centerY, int width, int height, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    glBegin(GL_QUADS);
    glVertex2i(centerX - width / 2, centerY + height / 2);
    glVertex2i(centerX - width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY + height / 2);
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}
void RenderBase::DrawSquare(int centerX, int centerY, int width, int height, float red, float green, float blue, float alpha) const
{
    glColor4f(red, green, blue, alpha);
    glBegin(GL_QUADS);
    glVertex2i(centerX - width / 2, centerY + height / 2);
    glVertex2i(centerX - width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY + height / 2);
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}

void RenderBase::DrawSquareBoundary(int centerX, int centerY, int width, int height, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    glBegin(GL_LINE_LOOP);
    glVertex2i(centerX - width / 2, centerY + height / 2);
    glVertex2i(centerX - width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY - height / 2);
    glVertex2i(centerX + width / 2, centerY + height / 2);
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}
void RenderBase::DrawLine(int centerX1, int centerY1, int centerX2, int centerY2, int width, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    glLineWidth(width);
    glBegin(GL_LINES);
    glVertex2i(centerX1, centerY1);
    glVertex2i(centerX2, centerY2);
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}
void RenderBase::DrawCircle(int centerX, int centerY, int diameter, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    glBegin(GL_TRIANGLE_FAN);
    for (int i = 0; i < 64; ++i)
    {
        double ang = YsPi * (double)i / 32.0;
        double s = sin(ang);
        double c = cos(ang);
        double x = centerX + diameter / 2 * c;
        double y = centerY + diameter / 2 * s;
        glVertex2d(x, y);
    }
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}

void RenderBase::DrawCircleBoundary(int centerX, int centerY, int diameter, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 64; ++i)
    {
        double ang = YsPi * (double)i / 32.0;
        double s = sin(ang);
        double c = cos(ang);
        double x = centerX + diameter / 2 * c;
        double y = centerY + diameter / 2 * s;
        glVertex2d(x, y);
    }
    glEnd();
    // glColor3f(0.0, 0.0, 0.0);
}
void RenderBase::DrawCubeBody(double x0, double y0, double z0, double x1, double y1, double z1, float red, float green, float blue) const
{
    glBegin(GL_QUADS);
    glColor3f(red, green, blue);

    glVertex3d(x0, y0, z0);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y1, z0);
    glVertex3d(x0, y1, z0);

    glVertex3d(x0, y0, z1);
    glVertex3d(x1, y0, z1);
    glVertex3d(x1, y1, z1);
    glVertex3d(x0, y1, z1);

    glVertex3d(x0, y0, z0);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y0, z1);
    glVertex3d(x0, y0, z1);

    glVertex3d(x0, y1, z0);
    glVertex3d(x1, y1, z0);
    glVertex3d(x1, y1, z1);
    glVertex3d(x0, y1, z1);

    glVertex3d(x0, y0, z0);
    glVertex3d(x0, y1, z0);
    glVertex3d(x0, y1, z1);
    glVertex3d(x0, y0, z1);

    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y1, z0);
    glVertex3d(x1, y1, z1);
    glVertex3d(x1, y0, z1);

    glDisable(GL_BLEND);
    glEnd();
}

void RenderBase::DrawCubeBoundary(double x0, double y0, double z0, double x1, double y1, double z1, float red, float green, float blue) const
{
    glColor3f(0, 0, 0);
    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y0, z0);
    glVertex3d(x1, y0, z0);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y1, z0);
    glVertex3d(x1, y1, z0);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y0, z1);
    glVertex3d(x1, y0, z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y1, z1);
    glVertex3d(x1, y1, z1);
    glEnd();


    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y0, z0);
    glVertex3d(x0, y1, z0);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y1, z0);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y0, z1);
    glVertex3d(x0, y1, z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x1, y0, z1);
    glVertex3d(x1, y1, z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y0, z0);
    glVertex3d(x0, y0, z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x1, y0, z0);
    glVertex3d(x1, y0, z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x0, y1, z0);
    glVertex3d(x0, y1, z1);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3d(x1, y1, z0);
    glVertex3d(x1, y1, z1);
    glEnd();

    // glColor3f(0.0, 0.0, 0.0);
}

void RenderBase::DrawFontWithTheSize(std::string fontSize, const char* c) const
{
    const char* c_fontsize = fontSize.c_str();

    if ("YsFont6x7" == fontSize)
    {
        YsGlDrawFontBitmap6x7(c);
    }
    else if ("YsFont6x8" == fontSize)
    {
        YsGlDrawFontBitmap6x8(c);
    }
    else if ("YsFont6x10" == fontSize)
    {
        YsGlDrawFontBitmap6x10(c);
    }
    else if ("YsFont7x10" == fontSize)
    {
        YsGlDrawFontBitmap7x10(c);
    }
    else if ("YsFont8x8" == fontSize)
    {
        YsGlDrawFontBitmap8x8(c);
    }
    else if ("YsFont8x12" == fontSize)
    {
        YsGlDrawFontBitmap8x12(c);
    }
    else if ("YsFont10x14" == fontSize)
    {
        YsGlDrawFontBitmap10x14(c);
    }
    else if ("YsFont12x16" == fontSize)
    {
        YsGlDrawFontBitmap12x16(c);
    }
    else if ("YsFont16x20" == fontSize)
    {
        YsGlDrawFontBitmap16x20(c);
    }
    else if ("YsFont16x24" == fontSize)
    {
        YsGlDrawFontBitmap16x24(c);
    }
    else if ("YsFont20x28" == fontSize)
    {
        YsGlDrawFontBitmap20x28(c);
    }
    else if ("YsFont20x32" == fontSize)
    {
        YsGlDrawFontBitmap20x32(c);
    }
    else if ("YsFont24x40" == fontSize)
    {
        YsGlDrawFontBitmap24x40(c);
    }
    else if ("YsFont28x44" == fontSize)
    {
        YsGlDrawFontBitmap28x44(c);
    }
    else if ("YsFont32x48" == fontSize)
    {
        YsGlDrawFontBitmap32x48(c);
    }
}

void RenderBase::DrawFontWithTheSize(std::string fontSize, double value) const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << value;
    std::string str = ss.str();
    const char* c = str.c_str();
    DrawFontWithTheSize(fontSize, c);
}


void RenderBase::DrawFontWithTheSize(std::string fontSize, const char* c, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    DrawFontWithTheSize(fontSize, c);
    // glColor3f(0.0, 0.0, 0.0);
}

void RenderBase::DrawFontWithTheSize(std::string fontSize, double value, float red, float green, float blue) const
{
    glColor3f(red, green, blue);
    DrawFontWithTheSize(fontSize, value);
    // glColor3f(0.0, 0.0, 0.0);
}