/* ********************************************************
 *   This file contains control algerithms that may be used 
 * in robotic arm control.
 * ********************************************************/

#include "control.hpp"
#include <stdio.h>
#include <unistd.h>

using namespace std;
using namespace cv;

//global variables
const vector<int> initValue {127,255,50,125,235,165,90};
vector<int> oldVals = initValue;
const int motorNumber = 7;


void fixStepMove(std::vector<int>& newVals,
                 UsbCAN& canDev,
                 int ID)
{
    if(oldVals.size()!=newVals.size())
    {
        printf("old and new value vectors do not have the same size\n");
        exit(-1);
    }

    while(1)
    {
        size_t satisfiedNum = 0;
        for(size_t i=0;i<motorNumber;i++)
        {
            if(newVals[i]-oldVals[i] > 0)
                oldVals[i] += 1;
            else if(newVals[i]-oldVals[i] < 0)
                oldVals[i] += -1;
            else
                satisfiedNum++;
        }

        VCI_CAN_OBJ can;
        generateFrame(can,oldVals,ID);
        canDev.transmit(&can,1);

        if(satisfiedNum == motorNumber)
            break;
        
        usleep(10*1000);
    }
}


void evenVelMove(std::vector<int>& newVals,
                 UsbCAN& canDev,
                 int ID)
{
    if(oldVals.size()!=newVals.size())
    {
        printf("old and new value vectors do not have the same size\n");
        exit(-1);
    }

    int changeVals[motorNumber];
    int maxChangeVal = 0;
    for(size_t i=0;i<motorNumber;i++)
    {
        changeVals[i] = newVals[i]-oldVals[i];
        maxChangeVal = (abs(changeVals[i])>maxChangeVal)?
            abs(changeVals[i]):maxChangeVal;
    }

    float stepPerChange[motorNumber];
    for(size_t i=0;i<motorNumber;i++)
    {
        if(changeVals[i]!=0)
        {
            stepPerChange[i] = 1.0f*maxChangeVal/abs(changeVals[i]);
            //normalize to +-one
            changeVals[i] /= abs(changeVals[i]);
        }
        else
            stepPerChange[i] = -1.0f;
    }

    int step[motorNumber] {0};
    VCI_CAN_OBJ canFrame;
    for(int i=0;i<maxChangeVal;i++)
    {
        for(int j=0;j<motorNumber;j++)
            if(stepPerChange[j]*step[j]<=i && stepPerChange[j]>=0)
            {
                oldVals[j]+=changeVals[j];
                step[j]++;
            }

        generateFrame(canFrame,oldVals,ID);
        canDev.transmit(&canFrame,1);
        usleep(10*1000);
    }

    oldVals = newVals;
    generateFrame(canFrame,oldVals,ID);
    canDev.transmit(&canFrame,1);
}


void reset2initPos(std::vector<int>& newVals,
                   UsbCAN& canDev,
                   int ID,
                   robot_arm::MOVEMENT_METHOD method)
{
    newVals = initValue;

    if(method == robot_arm::FIX_STEP)
        fixStepMove(newVals,canDev,ID);
    else
        evenVelMove(newVals,canDev,ID);
}


void move2desiredPos(double x,double y,
                     std::vector<int>& newVals,
                     TfNetwork& network,
                     UsbCAN& canDev,
                     int ID,
                     robot_arm::MOVEMENT_METHOD method)
{
    vector<double> inout{x,y};
    network.callFunction(inout,inout);
    newVals[1] = (int)inout[0];
    newVals[2] = (int)inout[1];
    
    if(method == robot_arm::FIX_STEP)
        fixStepMove(newVals,canDev,ID);
    else
        evenVelMove(newVals,canDev,ID);
}


double obstacleHeight(cv::Mat depthRaw,
                      float depthScale,
                      cv::Point2f targetPos,
                      int frontPixelOffset)
{
    const auto distMin = 0.45f;
    const auto distMax = 0.72f;
    const double a = 0.9596;
    const double b = -231.738;

    //280,180
    Mat roi = depthRaw(Rect(Point(280+frontPixelOffset,180),targetPos));

    for(auto y=0;y<roi.rows;y++)
    {
        auto depth = roi.ptr<uint16_t>(y);
        for(auto x=0;x<roi.cols;x++)
        {
            auto pixels_distance = depthScale * depth[x];
            if(pixels_distance > distMin && pixels_distance < distMax)
            {
                return a*(180+y)+b;
            }
        }
    }
    return a*targetPos.y+b;
}


double motor1moveAngle(Vec3d targetPos)
{
    const Vec3d origin(263,-7.2, 0);
    return atan((targetPos[1]-origin[1])/(targetPos[0]-origin[0]));
}


int motor1moveValue(Vec3d targetPos,double upperMmOffset)
{
    const double coeffs[] {-84.5,16.39,-81.97,123.9};
    const Vec3d origin(263,-7.2, 0);
    auto angle = atan((targetPos[1]-origin[1])/(targetPos[0]-upperMmOffset-origin[0]));
    auto returnVal = ((coeffs[0]*angle + coeffs[1])
                        *angle + coeffs[2])
                        *angle + coeffs[3];
    return (int)returnVal;
}

void getDetectImg(robot_arm::EVENT_FLAG& flag)
{
    usleep(100*1000);
    flag = robot_arm::TAKE_ROI;
    while(flag!=robot_arm::MISSION_OK);
}


void selfCalibration(ArucoMarker& frontMarker,
                     ArucoMarker& upperMarker,
                     double& frontMmOffset, double& upperMmOffset,
                     int& frontPixelOffset, int& upperPixelOffset,
                     UsbCAN& canDev,
                     int ID)
{
    const double frontCalibOrigin = -60.0;
    const double upperCalibOrigin = 70;
    const int frontPixelOrigin = 234;
    const int upperPixelOrigin = 508;
    vector<int> upperCalibValue {127,155,83,0,235,165,90};
    vector<int> temp;

    //calibrate front camera
    reset2initPos(temp,canDev,ID);
    usleep(500*1000);
    while(1)
    {
        if(frontMarker.index(5)!=-1)
        {
            frontMmOffset = frontMarker.offset_tVecs[frontMarker.index(5)][0]
                - frontCalibOrigin;
            frontPixelOffset = frontMarker.firstCorner(5).x - frontPixelOrigin;
            break;
        }
    }

    //calibrate upper camera
    evenVelMove(upperCalibValue,canDev,ID);
    usleep(500*1000);
    while(1)
    {
        if(upperMarker.index(5)!=-1)
        {
            upperMmOffset = upperMarker.offset_tVecs[upperMarker.index(5)][0]
                - upperCalibOrigin;
            upperPixelOffset = upperMarker.firstCorner(5).x - upperPixelOrigin;
            break;
        }
    }

    //return to initial position
    reset2initPos(temp,canDev,ID);
}

