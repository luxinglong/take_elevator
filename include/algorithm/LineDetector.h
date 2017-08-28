#pragma once

#include <iostream>
#include <vector>
#include <cmath>

#include "algorithm/core.hpp"

class LineDetector
{
public:
    void getHoughLine(const std::vector<TPoint2D> &refPoints, std::vector<HoughLine> &lines);
    void getHoughLine(const std::vector<LidarScanData> &new_scanData, std::vector<HoughLine> &lines);
    std::vector<HoughLine> lines;

protected:
    bool calculateHoughParameter(HoughLine &line);
    void nonlinearRegressionGN(HoughLine &line, std::vector<TPoint2D> &points);
    void protection(HoughLine &line, const std::vector<TPoint2D> &points);
    void getProjectedPoint(const HoughLine &line, const TPoint2D &point, TPoint2D &projectedPoint);

private:
    float maxAcceptGap;
    float maxAcceptDistance;
    int countThreshold;
};