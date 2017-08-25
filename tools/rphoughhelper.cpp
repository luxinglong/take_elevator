/* 
*  RoboPeak Project
*  Copyright 2009-2013
*
*  RoboPeak RPmini based SLAM Software Development Platform
*  Internal Test Only
*
*  Help funtions
*
*  Original version by Jacky Li
*/

#include "common.h"
#include <motionplan/algorithm/rpseedtracker.h>
#include <motionplan/algorithm/rphoughhelper.h>
#include "motionplan/algorithm/laserhelper.h"
#include "motionplan/algorithm/anglehelper.h"

#include <cmath>
//对自己的代码和知识进行管理，做过一遍的事情，就不要再去重复了
//代码管理： github
//知识管理： blog


class HoughLine
{
public:

private:
    struct line
    {
        TPoint2D start;
        TPoint2D end;
    };
    float rho;
    float theta;
    struct line
    float length;
    float sinTheta;
    float cosTheta;
};

class TPoint2D
{
public:
    TPoint2D(float x, float y);
    ~TPoint2D();
    float norm()
    {
        return sqrt(x*x + y*y);
    }
    TPoint2D operator-(TPoint2D& p0, TPoint2D& p1)
    {
        return TPoint2D(p0.x-p1.x, p0.y-p1.y);
    }
private:
    float x;
    float y;
};

namespace rp{ namespace algorithm { namespace slamsdp {

bool RpHoughHelper::calculateHoughParameter(HoughLine &line)
{
    float &rho = line.rho;
    float &theta = line.theta;
    TPoint2D &startP = line.line.start;
    TPoint2D &endP = line.line.end;
    line.length = (startP - endP).norm();

    if(startP == endP)
        return false;

    if(startP.x == endP.x) {
        theta = 0;
    } else if(startP.y == endP.y) {
        theta = M_PI/2;
    } else {
        float k = (startP.y - endP.y)/(startP.x - endP.x);
        theta = atan(-1/k);
    }

    //Hough transformation
    rho = startP.x * cos(theta) + startP.y * sin(theta);
    if(rho<0)
    {
        rho = -rho;
        theta += M_PI;
    }

    theta = rp::algorithm::math::AngleHelper::normAngle2(theta);

    line.cosTheta = cos(theta);
    line.sinTheta = sin(theta);
    float rho_check = endP.x * line.cosTheta + endP.y * line.sinTheta;
    float check = abs(rho-rho_check);
    if(check>0.1)
    {
        //should not run here, any time run into here denotes error actually
        return false;
    }

    return true;
}

void RpHoughHelper::nonlinearRegressionGN(HoughLine &line, std::vector<TPoint2D> &points)
{
    // please refer to confluence page for detailed information:
    // https://confluence.slamtec.com/pages/viewpage.action?pageId=2359298
    float theta = line.theta;
    float cosTheta = line.cosTheta;
    float sinTheta = line.sinTheta;
    Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
    Eigen::Vector2f dTr = Eigen::Vector2f::Zero();
    int size = points.size();
    for(int i=0; i<size; i+=2)
    {
        TPoint2D & point = points.at(i);
        Eigen::Vector2f partialDerivative;
        partialDerivative << point.x * (-sinTheta) + point.y * cosTheta, -1;  //Eigen重载了<<运算符，用来赋值
        float funVal = point.x * cosTheta + point.y * sinTheta - line.rho;

        dTr[0] += partialDerivative[0] * (-funVal);
        dTr[1] += partialDerivative[1] * (-funVal);

        H(0, 0) += partialDerivative[0] * partialDerivative[0];
        H(1, 1) += partialDerivative[1] * partialDerivative[1];

        H(0, 1) += partialDerivative[0] * partialDerivative[1];

        if(i==0) i = -1;
    }
    H(1, 0) = H(0, 1);

    Eigen::Vector2f searchDir(H.inverse() * dTr);
    if(mrpt::math::isNaN(searchDir[0]) || mrpt::math::isNaN(searchDir[1]))
    {
        searchDir.setZero();
    }
    line.theta += searchDir[0];
    line.rho   += searchDir[1];

    if(line.rho < 0)
    {
        line.rho = -line.rho;
        line.theta += M_PI;
    }

    line.cosTheta = cos(line.theta);
    line.sinTheta = sin(line.theta);
}

// 将points的首尾元素投影到line上，得到line的起始和终止点，并根据端点计算线段长度
void RpHoughHelper::projection(HoughLine &line, const std::vector<TPoint2D> &points)
{
    getProjectedPoint(line, points.front(), line.line.start);
    getProjectedPoint(line, points.back(), line.line.end);
    line.length = (line.line.start - line.line.end).norm();
}

// 将point投影到line上，并得到projectedPoint
void RpHoughHelper::getProjectedPoint(const HoughLine &line, const TPoint2D &point, TPoint2D &projectedPoint)
{
    float distance = point.x * line.cosTheta + point.y * line.sinTheta - line.rho;
    projectedPoint = point - TPoint2D(distance * line.cosTheta, distance * line.sinTheta);
}

// 判断投影点是在线段里还是线段外
bool RpHoughHelper::getProjectedPointWithinLine(const HoughLine &line, const TPoint2D &point, TPoint2D &projectedPoint, const CPose3D & robotPose)
{
    TPoint2D localProjectedPoint;
    getProjectedPoint(line, point, localProjectedPoint);
    TPoint2D vec1 = localProjectedPoint - line.line.start;
    TPoint2D vec2 = line.line.end - localProjectedPoint;
    float dotValue = vec1.x * vec2.x + vec1.y * vec2.y;
    if(dotValue > 0) // 线段里
    {
        projectedPoint = CPose2D(robotPose) + localProjectedPoint;
        return true;
    }
    return false;
}

// 
bool RpHoughHelper::isOutOfBound(HoughLine &houghLine, const TPoint2D & chkPoint, float extraThreshold)
{
    TPoint2D point = houghLine.oriLine.start - chkPoint;
    float chkRho = point.x * cos(houghLine.theta) + point.y * sin(houghLine.theta);
    if(chkRho < -abs(extraThreshold)) 
    {
        return true;
    }
    return false;
}

void RpHoughHelper::getLocalHoughLine(
    const std::vector<TPoint2D> &refPoints,
    std::vector<HoughLine> &lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    const size_t n = refPoints.size();

    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    int failCounter = 0;
    std::vector<TPoint2D> points;
    int backSpace = 0;
    for(size_t i=0; i<n; i++, backSpace++) 
    {
        int counter = points.size();
        TPoint2D point = refPoints[i];

        if(counter==0) {
            line.line.start = point;
            points.push_back(point);
            backSpace = 0;
        } else {
            float gapThreshold = max<float>(line.rho * 0.1, maxAcceptGap);
            if(counter<2) {
                float gap = (point - points.back()).norm();
                if(gap>gapThreshold) {
                    points.clear();
                    points.push_back(point);
                    failCounter = 0;
                    line.line.start = point;
                } else {
                    line.line.end = point;
                    calculateHoughParameter(line);
                    points.push_back(point);
                }
                backSpace = 0;
            } else {
                float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                float check = abs(line.rho-rho_check);
                float gap = (point - points.back()).norm();
                float rhoThreshold = max<float>(line.rho * 0.02, maxAcceptDistance);
                if(check>rhoThreshold || gap>gapThreshold) {
                    if(failCounter<1) {
                        failCounter++;
                    } else {
                        if(counter > countThreshold)
                        {
                            projection(line, points);
                            lines.push_back(line);
                        }

                        i -= (backSpace + 1);
                        failCounter = 0;
                        points.clear();
                    }
                } else {
                    failCounter = 0;
                    backSpace = 0;
                    line.line.end = point;
                    points.push_back(point);
                    for(int j=0; j<2; j++)
                    {
                        nonlinearRegressionGN(line, points);
                    }
                }
            }
        }
        if(i==n-1 && points.size()>countThreshold)
        {
            projection(line, points);
            lines.push_back(line);
        }
    }
}

bool RpHoughHelper::getHoughLinesByOrder(
    std::vector<HoughLine> &refLines,
    const std::vector<TPoint2D> &refPoints,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    bool result = false;
    std::vector<HoughLine> lines;
    getLocalHoughLine(refPoints, lines, countThreshold, maxAcceptDistance, maxAcceptGap);

    for(std::vector<HoughLine>::const_iterator it=lines.begin(); it!=lines.end(); ++it)
    {
        result = true;
        int size = refLines.size();
        if(size==0) {
            refLines.push_back(*it);
        } else if(size==1) {
            if(it->length <= refLines.front().length) {
                refLines.push_back(*it);
            } else {
                refLines.insert(refLines.begin(), *it);
            }
        } else {
            std::vector<HoughLine>::iterator former = refLines.begin();
            std::vector<HoughLine>::const_iterator latter = refLines.begin() + size - 1;
            if(it->length <= latter->length) {
                refLines.push_back(*it);
            } else {
                for(; former!=refLines.end(); former++)
                {
                    if(it->length > former->length)
                    {
                        refLines.insert(former, *it);
                        break;
                    }
                }
            }
        }
    }

    return result;
}

void RpHoughHelper::getHoughLineFromSeed(
    const BoundarySeedTracker & seedTracker,
    std::vector<HoughLine> &out_lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{

    std::vector<TPoint2D> points;
    HoughLine line;
    int failCounter = 0;
    out_lines.clear();

    for (size_t pos = seedTracker.getLoopStartPos(); pos < seedTracker.getSeeds().size(); ++pos)
    {
        TPoint2D currentPt = (TPoint2D)seedTracker.getSeeds()[pos].pose;

        int counter = points.size();

        if (counter == 0) {
            line.line.start = currentPt;
            points.push_back(currentPt);
        } else {
            float gapThreshold = max<float>(line.rho * 0.1, maxAcceptGap);
            if (counter<2) {
                float gap = (currentPt - points.back()).norm();
                if(gap>gapThreshold) {
                    points.clear();
                    points.push_back(currentPt);
                    failCounter = 0;
                    line.line.start = currentPt;
                } else {
                    line.line.end = currentPt;
                    calculateHoughParameter(line);
                    points.push_back(currentPt);
                }
            } else {
                float rho_check = currentPt.x * line.cosTheta + currentPt.y * line.sinTheta;
                float check = abs(line.rho-rho_check);
                float gap = (currentPt - points.back()).norm();
                float rhoThreshold = max<float>(line.rho * 0.02, maxAcceptDistance);
                if(check>rhoThreshold || gap>gapThreshold) {
                    if(failCounter<4) {
                        failCounter++;
                    } else {
                        if(counter > countThreshold)
                        {
                            projection(line, points);
                            out_lines.push_back(line);
                        }

                        pos -= 4;
                        failCounter = 0;
                        points.clear();
                    }
                } else {
                    failCounter = 0;
                    line.line.end = currentPt;
                    points.push_back(currentPt);
                    for(int j=0; j<2; j++)
                    {
                        nonlinearRegressionGN(line, points);
                    }
                }
            }
        }
        if (pos==seedTracker.getSeeds().size()-1 && points.size()>countThreshold)
        {
            projection(line, points);
            out_lines.push_back(line);
        }
    }
}

void RpHoughHelper::getLocalHoughLine(
    const std::vector<LidarScanData> &new_scanData,
    std::vector<HoughLine> &lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    const size_t n = new_scanData.size();

    float distance = 0;
    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    line.rho = 0;
    int failCounter = 0;
    std::vector<TPoint2D> points;
    int backSpace = 0;
    for(size_t i=0; i<n; i++, backSpace++) 
    {
        LidarScanData scanData = new_scanData[i];

        //float angle = M_PI - inc_ang * i;
        float angle = scanData.angle;

        if(scanData.valid)
        {
            distance = scanData.dist;
            if(distance<0)
            {
                continue;
            }

            int counter = points.size();
            TPoint2D point(distance*cos(angle), distance*sin(angle));

            if(counter==0) {
                line.line.start = point;
                points.push_back(point);
                backSpace = 0;
            } else {
                float gapThreshold = max<float>(line.rho * 0.1, maxAcceptGap);
                if(counter<2) {
                    float gap = (point - points.back()).norm();
                    if(gap>gapThreshold) {
                        points.clear();
                        points.push_back(point);
                        failCounter = 0;
                        line.line.start = point;
                    } else {
                        line.line.end = point;
                        calculateHoughParameter(line);
                        points.push_back(point);
                    }
                    backSpace = 0;
                } else {
                    float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                    float check = abs(line.rho-rho_check);
                    float gap = (point - points.back()).norm();
                    float rhoThreshold = max<float>(line.rho * 0.02, maxAcceptDistance);
                    if(check>rhoThreshold || gap>gapThreshold) {
                        if(failCounter<1) {
                            failCounter++;
                        } else {
                            if(counter > countThreshold)
                            {
                                projection(line, points);
                                lines.push_back(line);
                            }

                            i -= (backSpace + 1);
                            failCounter = 0;
                            points.clear();
                        }
                    } else {
                        failCounter = 0;
                        backSpace = 0;
                        line.line.end = point;
                        points.push_back(point);
                        for(int j=0; j<2; j++)
                        {
                            nonlinearRegressionGN(line, points);
                        }
                    }
                }
            }
        }
        if(i==n-1 && points.size()>countThreshold)
        {
            projection(line, points);
            lines.push_back(line);
        }
    }
}

void RpHoughHelper::getLocalHoughLine(
    const std::vector<SeedData> &seedDataes,
    const CPose3D &curRobotPose,
    std::vector<HoughLine> &lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    const size_t n = seedDataes.size();

    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    int failCounter = 0;
    std::vector<TPoint2D> points;
    int backSpace = 0;
    for(size_t i=0; i<n; i++, backSpace++) 
    {
        SeedData seedData = seedDataes[i];

        int counter = points.size();
        TPoint2D point = -CPose2D(curRobotPose) + TPoint2D(seedData.pose);

        if(counter==0) {
            line.line.start = point;
            points.push_back(point);
            backSpace = 0;
        } else {
            float gapThreshold = max<float>(line.rho * 0.1, maxAcceptGap);
            if(counter<2) {
                float gap = (point - points.back()).norm();
                if(gap>gapThreshold) {
                    points.clear();
                    points.push_back(point);
                    failCounter = 0;
                    line.line.start = point;
                } else {
                    line.line.end = point;
                    calculateHoughParameter(line);
                    points.push_back(point);
                }
                backSpace = 0;
            } else {
                float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                float check = abs(line.rho-rho_check);
                float gap = (point - points.back()).norm();
                float rhoThreshold = max<float>(line.rho * 0.02, maxAcceptDistance);
                if(check>rhoThreshold || gap>gapThreshold) {
                    if(failCounter<1) {
                        failCounter++;
                    } else {
                        if(counter > countThreshold)
                        {
                            projection(line, points);
                            lines.push_back(line);
                        }

                        i -= (backSpace + 1);
                        failCounter = 0;
                        points.clear();
                    }
                } else {
                    failCounter = 0;
                    backSpace = 0;
                    line.line.end = point;
                    points.push_back(point);
                    for(int j=0; j<2; j++)
                    {
                        nonlinearRegressionGN(line, points);
                    }
                }
            }
        }
        if(i==n-1 && points.size()>countThreshold)
        {
            projection(line, points);
            lines.push_back(line);
        }
    }
}

void RpHoughHelper::getHoughLine(
    const std::vector<LidarScanData> &new_scanData,
    const CPose3D &curRobotPose,
    float margin,
    std::vector<HoughLine> &lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    getLocalHoughLine(new_scanData, lines, countThreshold, maxAcceptDistance, maxAcceptGap);
    formulateLines(lines, curRobotPose, margin);
}

void RpHoughHelper::getHoughLine(
    CObservation2DRangeScanPtr lidarScan,
    const CPose3D &curRobotPose,
    float margin,
    std::vector<HoughLine> &lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    const size_t n = lidarScan->scan.size();

    std::vector<LidarScanData> scanDatas;
    for(size_t i=0; i<n; i++) 
    {
        LidarScanData scanData;
        scanData.dist = lidarScan->scan[i];
        if (lidarScan->hasAngle) {
            scanData.angle = lidarScan->angle[i];
        } else {
            scanData.angle = rp::algorithm::math::AngleHelper::normAngle(LaserRangerHelper::idx2Angle(lidarScan, i) - M_PI);//May wrong but fortunately not used any longer
        }
        if (!lidarScan->validRange[i]) {
            scanData.valid = false;
        } else {
            scanData.valid = true;
        }
        float angle = scanData.angle;
        scanDatas.push_back(scanData);
    }

    getHoughLine(scanDatas, curRobotPose, margin, lines, countThreshold, maxAcceptDistance, maxAcceptGap);
}

void RpHoughHelper::getHoughLine(
    const std::vector<SeedData> &seedDataes,
    const CPose3D &curRobotPose,
    float margin,
    std::vector<HoughLine> &lines,
    int countThreshold,
    float maxAcceptDistance,
    float maxAcceptGap)
{
    getLocalHoughLine(seedDataes, curRobotPose, lines, countThreshold, maxAcceptDistance, maxAcceptGap);
    formulateLines(lines, curRobotPose, margin);
}

// extract the room heading via voting mechanism
bool RpHoughHelper::getMostLikelyEnvironmentHeading (
    float & heading,
    const std::vector<LidarScanData> &new_scanData,
    const CPose3D &curRobotPose,
    float margin,
    int countThreshold,
    float headingSimilarityThreshold)
{
    std::vector<HoughLine> lines;
    RpHoughHelper::getHoughLine (new_scanData, curRobotPose, margin, lines, countThreshold);
                          
    std::vector<std::pair<float, float>> votepool;
                        // ^       ^
                        // |       +--- vote counter
                        // +----------- heading

    // stage 1: voting...
    int largestCandidateID = -1;
    float largestVote = 0;

    for (std::vector<HoughLine>::iterator it = lines.begin (); it != lines.end (); ++it)
    {
        float currentlength = it->length;
        float currentHeading = rp::algorithm::math::AngleHelper::normAngle2(it->theta);

        // search whether similar voting exists...
        int similarVoteId = -1;
        float lastSimilarity = 1000;
        for (size_t pos = 0; pos < votepool.size (); ++pos)
        {
            float voteSimilarity = (rp::algorithm::math::AngleHelper::angleDiff2(currentHeading, votepool[pos].first));
            if (fabs(voteSimilarity) < fabs(lastSimilarity)) {
                lastSimilarity = voteSimilarity;
                similarVoteId = pos;
            }
        }

        if ( (similarVoteId == -1) || (fabs(lastSimilarity) > headingSimilarityThreshold)) {
            // push a new candidate ...
            votepool.push_back(std::pair<float,float>(currentHeading, currentlength));
            similarVoteId = votepool.size()-1;
        } else {
            // fuse with the existing one...

            std::pair<float, float> & fuseTarget = votepool[similarVoteId];
            float newTotalVote = fuseTarget.second + currentlength;
            float newHeading = (fuseTarget.first * fuseTarget.second + (fuseTarget.first + lastSimilarity) * currentlength)/ newTotalVote;
            fuseTarget.first = rp::algorithm::math::AngleHelper::normAngle2 (newHeading);
            fuseTarget.second = newTotalVote;
        }

        if (votepool[similarVoteId].second > largestVote) {
            largestVote = votepool[similarVoteId].second;
            largestCandidateID = similarVoteId;
        }
    }

    if (largestCandidateID == -1) {
        heading = 0;
        return false;
    } else {
        heading = votepool[largestCandidateID].first;
        return true;
    }
}

bool RpHoughHelper::getLongestHoughLine(
    HoughLine &line,
    const std::vector<LidarScanData> &new_scanData,
    const CPose3D &curRobotPose,
    float margin,
    int countThreshold)
{
    bool result = false;
    std::vector<HoughLine> lines;
    RpHoughHelper::getHoughLine(new_scanData, curRobotPose, margin, lines, countThreshold);

    float maxLength = 0;
    HoughLine maxLenHoughLine;
    for(std::vector<HoughLine>::iterator it=lines.begin(); it!=lines.end(); ++it)
    {
        float length = it->length;
        if(length > maxLength)
        {
            maxLenHoughLine = *it;
            maxLength = length;
            result = true;
        }
    }
    if(result)
    {
        line = maxLenHoughLine;
    }

    return result;
}

bool RpHoughHelper::getLongestHoughLine(
    HoughLine &line,
    CObservation2DRangeScanPtr lidarScan,
    const CPose3D &curRobotPose,
    float margin,
    int countThreshold)
{
    bool result = false;
    std::vector<HoughLine> lines;
    RpHoughHelper::getHoughLine(lidarScan, curRobotPose, margin, lines, countThreshold);

    float maxLength = 0;
    HoughLine maxLenHoughLine;
    for(std::vector<HoughLine>::iterator it=lines.begin(); it!=lines.end(); ++it)
    {
        float length = (it->line.start - it->line.end).norm();
        if(length > maxLength)
        {
            maxLenHoughLine = *it;
            maxLength = length;
            result = true;
        }
    }
    if(result)
    {
        line = maxLenHoughLine;
    }

    return result;
}

bool RpHoughHelper::getLongestHoughLine(
    HoughLine &line,
    const std::vector<SeedData> &seedDataes,
    const CPose3D &curRobotPose,
    float margin,
    int countThreshold)
{
    bool result = false;
    std::vector<HoughLine> lines;
    RpHoughHelper::getHoughLine(seedDataes, curRobotPose, margin, lines, countThreshold, theApp.applicationConfig.targetConfig.base.robot_size/4, theApp.applicationConfig.targetConfig.base.robot_size * 2);

    float maxLength = 0;
    HoughLine maxLenHoughLine;
    for(std::vector<HoughLine>::iterator it=lines.begin(); it!=lines.end(); ++it)
    {
        float length = (it->line.start - it->line.end).norm();
        if(length > maxLength)
        {
            maxLenHoughLine = *it;
            maxLength = length;
            result = true;
        }
    }
    if(result)
    {
        line = maxLenHoughLine;
    }

    return result;
}

void RpHoughHelper::formulateLines(
    std::vector<HoughLine> &lines,
    const CPose3D &curRobotPose, 
    float margin)
{
    for(std::vector<HoughLine>::iterator it=lines.begin(); it!=lines.end(); ++it)
    {
        HoughLine &lineRef = *it;
        formulateLine(lineRef, curRobotPose, margin);
    }
}

void RpHoughHelper::formulateLine(
    HoughLine &line, 
    const CPose3D &curRobotPose, 
    float margin)
{
    if(margin!=0)
    {
        TPoint2D marginP(margin*cos(line.theta), margin*sin(line.theta));
        line.line.start -= marginP;
        line.line.end   -= marginP;
    }

    line.oriLine.start = CPose2D(curRobotPose) + line.line.start;
    line.oriLine.end   = CPose2D(curRobotPose) + line.line.end;
}

bool RpHoughHelper::isDepartureFromLine(
    HoughLine &line, 
    TPoint2D &point, 
    float rhoThreshold)
{
    float rho_check = point.x * cos(line.theta) + point.y * sin(line.theta);
    float check = abs(line.rho-rho_check);
    return check>rhoThreshold;
}

bool RpHoughHelper::isHoughLine(
        const TPoint2D & startP, 
        const TPoint2D & endP, 
        const TPoint2D & point,
        float rhoThreshold)
{
    HoughLine line;
    line.line.start = startP;
    line.line.end = endP;
    if(calculateHoughParameter(line))
    {
        float rho_check = point.x * cos(line.theta) + point.y * sin(line.theta);
        float check = abs(line.rho-rho_check);
        if(check <= rhoThreshold) 
        {
            return true;
        } 
    }
    return false;
}

bool RpHoughHelper::isCrossWith(const HoughLine & line1, const HoughLine & line2)
{
    if(line1.oriLine.start==line2.oriLine.start && line1.oriLine.end==line2.oriLine.end)
    {
        return true;
    }

    TPoint2D vec_ab = line1.oriLine.start - line1.oriLine.end;
    TPoint2D vec_cd = line2.oriLine.start - line2.oriLine.end;

    TPoint2D vec_ac = line1.oriLine.start - line2.oriLine.start;
    TPoint2D vec_ad = line1.oriLine.start - line2.oriLine.end;
    TPoint2D vec_ca = line2.oriLine.start - line1.oriLine.start;
    TPoint2D vec_cb = line2.oriLine.start - line1.oriLine.end;

    float abXac = vec_ab.x * vec_ac.y - vec_ab.y * vec_ac.x;
    float abXad = vec_ab.x * vec_ad.y - vec_ab.y * vec_ad.x;
    float cdXca = vec_cd.x * vec_ca.y - vec_cd.y * vec_ca.x;
    float cdXcb = vec_cd.x * vec_cb.y - vec_cd.y * vec_cb.x;

    if(abXac * abXad<0 && cdXca * cdXcb<0)
    {
        return true;
    }
    return false;
}

bool RpHoughHelper::getIntersectPoint(const HoughLine & line1, const HoughLine & line2, TPoint2D & intersectedPoint, bool segmentFlag)
{
    Eigen::Matrix2f matrix;
    matrix << line1.oriLine.start.x - line1.oriLine.end.x, line2.oriLine.end.x - line2.oriLine.start.x,
              line1.oriLine.start.y - line1.oriLine.end.y, line2.oriLine.end.y - line2.oriLine.start.y;

    Eigen::Vector2f vec;
    vec << line2.oriLine.end.x - line1.oriLine.end.x, line2.oriLine.end.y - line1.oriLine.end.y;

    Eigen::Vector2f rst = matrix.inv() * vec;

    bool result = segmentFlag? (rst[0]>0 && rst[0]<1 && rst[1]>0 && rst[1]<1) : true;

    if(result)
    {
        intersectedPoint.x = line1.oriLine.end.x + rst[0] * (line1.oriLine.start.x - line1.oriLine.end.x);
        intersectedPoint.y = line1.oriLine.end.y + rst[0] * (line1.oriLine.start.y - line1.oriLine.end.y);
    }

    return result;
}

void RpHoughHelper::pointProjection(TPoint2D &point, float rho, float theta)
{
    float relativeRho = rho - (point.x * cos(theta) + point.y * sin(theta));
    point += TPoint2D(relativeRho * cos(theta), relativeRho * sin(theta));

    //TEST
    float rhoTest = rho - (point.x * cos(theta) + point.y * sin(theta));
}

float RpHoughHelper::getCosAngleBetween(const HoughLine & line1, const HoughLine & line2)
{
    TPoint2D vec1 = line1.oriLine.end - line1.oriLine.start;
    TPoint2D vec2 = line2.oriLine.end - line2.oriLine.start;
    return (vec1.x * vec2.x + vec1.y * vec2.y)/vec1.norm()/vec2.norm();
}

float RpHoughHelper::globalPointToLineDistance(const TPoint2D & point, const HoughLine & line)
{
    TPoint2D vec1 = point - line.oriLine.start;
    TPoint2D vec2 = line.oriLine.end - line.oriLine.start;
    float distance = abs(vec1.x * vec2.y - vec1.y * vec2.x)/vec2.norm();
    return distance;
}

bool RpHoughHelper::isGlobalPointWithinLine(const TPoint2D & point, const HoughLine & line)
{
    TPoint2D vec1 = point - line.oriLine.start;
    TPoint2D vec2 = line.oriLine.start - line.oriLine.end;

    float dotValue1 = vec1.x * vec2.x + vec1.y * vec2.y; 

    TPoint2D vec3 = point - line.oriLine.end;
    TPoint2D vec4 = line.oriLine.end - line.oriLine.start;

    float dotValue2 = vec3.x * vec4.x + vec3.y * vec4.y;

    if(dotValue1<=0 && dotValue2<=0)
    {
        return true;
    }

    return false;
}

void RpHoughHelper::normalFilterData(std::vector<LidarScanData> &rawScanData)
{
    rp::slamware::config::SlamwareLidarNormalFilter normalFilterConfig = theApp.applicationConfig.targetConfig.lidar.normal_filter;

    if(! normalFilterConfig.enable) return;

    float maxAcceptGap     = normalFilterConfig.max_gap_threshold;
    float rhoBiasThreshold = normalFilterConfig.max_rho_bias_threshold;
    int maxFailCounter     = normalFilterConfig.max_failure_count;
    int backSpace          = normalFilterConfig.back_space_count;
    int countThreshold     = normalFilterConfig.min_points_count_threshold;
    float maxRho           = normalFilterConfig.max_rho_threshold;


    const size_t n = rawScanData.size();

    float distance = 0;
    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    std::vector<TPoint2D> points;
    std::map<int, int> filterMap;
    int start = 0;
    int failCounter = 0;
    for(size_t i=0; i<n; i++, backSpace++)
    {
        LidarScanData & scanData = rawScanData[i];

        float angle = scanData.angle;

        if(scanData.valid)
        {
            distance = scanData.dist;
            if(distance<0)
            {
                continue;
            }

            int counter = points.size();
            float cos_angle = rp::algorithm::math::fixpoint::FastSinCos::fast_cos_normed(angle);
            float sin_angle = rp::algorithm::math::fixpoint::FastSinCos::fast_sin_normed(angle);

            TPoint2D point(distance*cos_angle, distance*sin_angle);

            if(counter==0) {
                line.line.start = point;
                points.push_back(point);
                backSpace = 0;
                start = i;
            } else {
                float gapThreshold = max<float>(line.rho * 0.1, maxAcceptGap);
                if(counter<2) {
                    float gap = (point - points.back()).norm();
                    if(gap>gapThreshold) {
                        points.clear();
                        points.push_back(point);
                        failCounter = 0;
                        line.line.start = point;
                        start = i;
                    } else {
                        line.line.end = point;
                        calculateHoughParameter(line);
                        points.push_back(point);
                    }
                    backSpace = 0;
                } else {
                    float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                    float check = abs(line.rho-rho_check);
                    float gap = (point - points.back()).norm();
                    float rhoThreshold = max<float>(line.rho * 0.02, rhoBiasThreshold);
                    if(check>rhoThreshold || gap>gapThreshold) {
                        if(failCounter<maxFailCounter) {
                            failCounter++;
                        } else {
                            if(counter >= countThreshold)
                            {
                                projection(line, points);
                                if(line.rho <= maxRho && (line.line.start.x * line.line.end.x + line.line.start.y * line.line.end.y) > 0)
                                {
                                    filterMap[start] = (i - backSpace);
                                }
                            }

                            i -= (backSpace + 1);
                            failCounter = 0;
                            points.clear();
                        }
                    } else {
                        failCounter = 0;
                        backSpace = 0;
                        line.line.end = point;
                        points.push_back(point);
                        for(int j=0; j<2; j++)
                        {
                            nonlinearRegressionGN(line, points);
                        }
                    }
                }
            }
        }
        if(i==n-1 && points.size()>=std::max<int>(2, countThreshold))
        {
            projection(line, points);
            if(line.rho <= maxRho && (line.line.start.x * line.line.end.x + line.line.start.y * line.line.end.y) > 0)
            {
                filterMap[start] = (i - backSpace);
            }
        }
    }

    for(std::map<int, int>::iterator it = filterMap.begin(); it!=filterMap.end(); it++)
    {
        int start = it->first;
        int end = it->second/* - 1*/;
        /*if(rawScanData[start].dist > rawScanData[end].dist) {
            end -= 1;
        } else {
            start += 1;
        }*/
        for(int i=start; i<end; i++)
        {
            rawScanData[i].valid = false;
        }
    }
}

void RpHoughHelper::fitFilterData(std::vector<LidarScanData> &rawScanData, bool angleNeedTrans, float maxAcceptDistance)
{
    rp::slamware::config::SlamwareLidarFitFilter fitFilterConfig = theApp.applicationConfig.targetConfig.lidar.fit_filter;

    if(! fitFilterConfig.enable) return;

    float maxAcceptGap = theApp.applicationConfig.targetConfig.base.robot_size/2;
    const size_t n = rawScanData.size();

    float distance = 0;
    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    line.rho = 0;
    int failCounter = 0;
    std::vector<TPoint2D> points;
    int backSpace = 0;
    int countThreshold = 4;
    int start = 0;
    for(size_t i=0; i<n; i++, backSpace++)
    {
        LidarScanData & scanData = rawScanData[i];

        float angle = scanData.angle;
        if(angleNeedTrans)
        {
            angle = M_PI - scanData.angle;
        }

        if(scanData.valid)
        {
            distance = scanData.dist;
            if(distance<0)
            {
                continue;
            }

            int counter = points.size();
            TPoint2D point(distance*cos(angle), distance*sin(angle));

            if(counter==0) {
                line.line.start = point;
                points.push_back(point);
                backSpace = 0;
                start = i;
            } else {
                float gapThreshold = max<float>(line.rho * 0.1, maxAcceptGap);
                if(counter<2) {
                    float gap = (point - points.back()).norm();
                    if(gap>gapThreshold) {
                        points.clear();
                        points.push_back(point);
                        failCounter = 0;
                        line.line.start = point;
                        start = i;
                    } else if(gap>1e-5) {
                        line.line.end = point;
                        calculateHoughParameter(line);
                        points.push_back(point);
                    }
                    backSpace = 0;
                } else {
                    float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                    float check = abs(line.rho-rho_check);
                    float gap = (point - points.back()).norm();
                    float rhoThreshold = max<float>(line.rho * 0.01, maxAcceptDistance);
                    if(check>rhoThreshold || gap>gapThreshold) {
                        if(gap>gapThreshold)
                        {
                            failCounter = 4;
                        }

                        if(failCounter<4) {
                            failCounter++;
                        } else {
                            if(counter >= countThreshold)
                            {
                                projection(line, points);
                                fitPoints(rawScanData, angleNeedTrans, start, i-backSpace, line);
                            }

                            i -= (backSpace + 1);
                            failCounter = 0;
                            points.clear();
                        }
                    } else {
                        failCounter = 0;
                        backSpace = 0;
                        line.line.end = point;
                        points.push_back(point);
                        for(int j=0; j<2; j++)
                        {
                            nonlinearRegressionGN(line, points);
                        }
                    }
                }
            }
        }
        if(i==n-1 && points.size()>=std::max<int>(2, countThreshold))
        {
            projection(line, points);
            fitPoints(rawScanData, angleNeedTrans, start, i-backSpace, line);
        }
    }
}

void RpHoughHelper::fitPoints(std::vector<LidarScanData> &rawScanData, bool angleNeedTrans, int start, int end, const HoughLine & line, float maxAcceptDistance)
{
    float maxAcceptGap = theApp.applicationConfig.targetConfig.base.robot_size/2;
    float distance = 0;
    for(int i=start; i<=end; i++)
    {
        LidarScanData & scanData = rawScanData[i];

        float angle = scanData.angle;
        if(angleNeedTrans)
        {
            angle = M_PI - scanData.angle;
        }

        if(scanData.valid)
        {
            distance = scanData.dist;
            if(distance<0)
            {
                continue;
            }

            TPoint2D point(distance*cos(angle), distance*sin(angle));
            float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
            float check = abs(line.rho-rho_check);
            float rhoThreshold = max<float>(line.rho * 0.01, maxAcceptDistance);
            if(check > rhoThreshold)
            {
                continue;
            }
            TPoint2D projectedPoint;
            RpHoughHelper::getProjectedPoint(line, point, projectedPoint);
            scanData.dist = projectedPoint.norm();
        }
    }
}

void RpHoughHelper::lengthFilterData(
    std::vector<LidarScanData> &rawScanData,
    int countThreshold,
    float rhoBiasThreshold,
    float maxAcceptGap,
    float lengthThreshold,
    bool lowPass)
{
    int maxFailCounter     = 0;
    int backSpace          = 0;

    const size_t n = rawScanData.size();

    float distance = 0;
    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    std::vector<TPoint2D> points;
    std::map<int, int> filterMap;
    int start = 0;
    int failCounter = 0;
    for(size_t i=0; i<n; i++, backSpace++)
    {
        LidarScanData & scanData = rawScanData[i];

        float angle = scanData.angle;

        if(scanData.valid)
        {
            distance = scanData.dist;
            if(distance<0)
            {
                continue;
            }

            int counter = points.size();
            float cos_angle = rp::algorithm::math::fixpoint::FastSinCos::fast_cos_normed(angle);
            float sin_angle = rp::algorithm::math::fixpoint::FastSinCos::fast_sin_normed(angle);
            TPoint2D point(distance*cos_angle, distance*sin_angle);

            if(counter==0) {
                line.line.start = point;
                points.push_back(point);
                backSpace = 0;
                start = i;
            } else {
                float gapThreshold = maxAcceptGap;
                if(counter<2) {
                    float gap = (point - points.back()).norm();
                    if(gap>gapThreshold) {
                        points.clear();
                        points.push_back(point);
                        failCounter = 0;
                        line.line.start = point;
                        start = i;
                    } else {
                        line.line.end = point;
                        calculateHoughParameter(line);
                        points.push_back(point);
                    }
                    backSpace = 0;
                } else {
                    float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                    float check = abs(line.rho-rho_check);
                    float gap = (point - points.back()).norm();
                    float rhoThreshold = rhoBiasThreshold;
                    if(check>rhoThreshold || gap>gapThreshold) {
                        if(failCounter<maxFailCounter) {
                            failCounter++;
                        } else {
                            if(counter >= countThreshold)
                            {
                                projection(line, points);
                                if(lowPass && line.length > lengthThreshold || !lowPass && line.length < lengthThreshold)
                                {
                                    filterMap[start] = (i - backSpace);
                                }
                            }

                            i -= (backSpace + 1);
                            failCounter = 0;
                            points.clear();
                        }
                    } else {
                        failCounter = 0;
                        backSpace = 0;
                        line.line.end = point;
                        points.push_back(point);
                        for(int j=0; j<2; j++)
                        {
                            nonlinearRegressionGN(line, points);
                        }
                    }
                }
            }
        }
        if(i==n-1 && points.size()>=std::max<int>(2, countThreshold))
        {
            projection(line, points);
            if(lowPass && line.length > lengthThreshold || !lowPass && line.length < lengthThreshold)
            {
                filterMap[start] = (i - backSpace);
            }
        }
    }

    for(std::map<int, int>::iterator it = filterMap.begin(); it!=filterMap.end(); it++)
    {
        int start = it->first;
        int end = it->second;
        for(int i=start; i<end; i++)
        {
            rawScanData[i].valid = false;
        }
    }
}

void RpHoughHelper::getFilterMapAndLines(
    const std::vector<LidarScanData> &scanDatas,
    const CPose3D &curRobotPose,
    float margin,
    std::vector<HoughLine> &lines,
    std::map<int, int> & filterMap,
    int countThreshold,
    float rhoBiasThreshold,
    float maxAcceptGap)
{
    int maxFailCounter     = 0;
    int backSpace          = 0;

    const size_t n = scanDatas.size();

    float distance = 0;
    TPoint2D startP(0, 0), endP(0, 0);
    HoughLine line;
    std::vector<TPoint2D> points;
    int start = 0;
    int failCounter = 0;
    for(size_t i=0; i<n; i++, backSpace++)
    {
        const LidarScanData & scanData = scanDatas[i];

        float angle = scanData.angle;

        if(scanData.valid)
        {
            distance = scanData.dist;
            if(distance<0)
            {
                continue;
            }

            int counter = points.size();
            TPoint2D point(distance*cos(angle), distance*sin(angle));

            if(counter==0) {
                line.line.start = point;
                points.push_back(point);
                backSpace = 0;
                start = i;
            } else {
                float gapThreshold = maxAcceptGap;
                if(counter<2) {
                    float gap = (point - points.back()).norm();
                    if(gap>gapThreshold) {
                        points.clear();
                        points.push_back(point);
                        failCounter = 0;
                        line.line.start = point;
                        start = i;
                    } else {
                        line.line.end = point;
                        calculateHoughParameter(line);
                        points.push_back(point);
                    }
                    backSpace = 0;
                } else {
                    float rho_check = point.x * line.cosTheta + point.y * line.sinTheta;
                    float check = abs(line.rho-rho_check);
                    float gap = (point - points.back()).norm();
                    float rhoThreshold = rhoBiasThreshold;
                    if(check>rhoThreshold || gap>gapThreshold) {
                        if(failCounter<maxFailCounter) {
                            failCounter++;
                        } else {
                            if(counter >= countThreshold)
                            {
                                projection(line, points);
                                filterMap[start] = (i - backSpace);
                                lines.push_back(line);
                            }

                            i -= (backSpace + 1);
                            failCounter = 0;
                            points.clear();
                        }
                    } else {
                        failCounter = 0;
                        backSpace = 0;
                        line.line.end = point;
                        points.push_back(point);
                        for(int j=0; j<2; j++)
                        {
                            nonlinearRegressionGN(line, points);
                        }
                    }
                }
            }
        }
        if(i==n-1 && points.size()>=std::max<int>(2, countThreshold))
        {
            projection(line, points);
            filterMap[start] = (i - backSpace);
            lines.push_back(line);
        }
    }
    formulateLines(lines, curRobotPose, margin);
}

}}}
