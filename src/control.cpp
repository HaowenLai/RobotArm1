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
vector<int> initValue {127,255,50,125,235,165,100};
vector<int> oldVals = initValue;


void fixStepMove(std::vector<int>& newVals,
                 UsbCAN& canDev,
                 int ID,
                 int step)
{
    if(oldVals.size()!=newVals.size())
    {
        printf("old and new value vectors do not have the same size\n");
        return;
    }

    while(1)
    {
        size_t satisfiedNum = 0;
        for(size_t i=0;i<oldVals.size();i++)
        {
            if(newVals[i]-oldVals[i] >= step)
                oldVals[i] += step;
            else if(newVals[i]-oldVals[i] <= -step)
                oldVals[i] += -step;
            else
                satisfiedNum++;
        }

        VCI_CAN_OBJ can;
        generateFrame(can,oldVals,ID);
        canDev.transmit(&can,1);

        if(satisfiedNum == oldVals.size())
            break;
        
        usleep(10*1000);
    }
}

void reset2initPos(std::vector<int>& newVals,
                   UsbCAN& canDev,
                   int ID,
                   int step)
{
    newVals = initValue;
    fixStepMove(initValue,canDev,ID,step);
}


void move2desiredPos(double x,double y,
                     std::vector<int>& newVals,
                     TfNetwork& network,
                     UsbCAN& canDev,
                     int ID,
                     int step)
{
    vector<double> inout{x,y};
    network.callFunction(inout,inout);
    newVals[1] = (int)inout[0];
    newVals[2] = (int)inout[1];
    fixStepMove(newVals,canDev,ID);
}


double obstacleHeight(cv::Mat depthRaw,
                      float depthScale,
                      cv::Point2f targetPos)
{
    const auto distMin = 0.53f;
    const auto distMax = 0.63f;
    const double a = 0.8793;
    const double b = -192.331;

    //150,180
    Mat roi = depthRaw(Rect(Point(150,180),targetPos));

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
    const Vec3d origin(-249.5, -31.7, 0);
    return atan((targetPos[1]-origin[1])/(targetPos[0]-origin[0]));
}
