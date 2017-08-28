#pragma once

#include <cmath>

// 霍夫线段类
class HoughLine
{
private:
    struct line
    {
        TPoint2D start;
        TPoint2D end;
    };
    struct oriLine
    {
        TPoint2D start;
        TPoint2D end;
    };
    float rho;
    float theta;
    float length;
    float sinTheta;
    float cosTheta;
};

//点类
class TPoint2D
{
public:
    TPoint2D(float x, float y);
    ~TPoint2D();
    inline float norm() const
    {
        return sqrt(x*x + y*y);
    }
    TPoint2D operator -(TPoint2D& p) const
    {
        return TPoint2D(x-p.x, y-p.y);
    }
    bool operator ==(TPoint2D& p) const
    {
        return (x==p.x && y==p.y);
    }
private:
    float x;
    float y;
};

class LidarScanData
{
private:
    float angle;
    float dist;
    float valid;
};