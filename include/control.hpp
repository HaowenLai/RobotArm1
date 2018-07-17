/* ********************************************************
 *   This file contains control algerithms that may be used 
 * in robotic arm control.
 * @Author : Derek Lai
 * @Date   : 2018/7/4
 * @Version: v2.5
 * Copyright(c) All right reserved
 * ********************************************************/

#ifndef __CONTROL_HPP__
#define __CONTROL_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "ArucoMarker.hpp"
#include "UsbCAN.hpp"
#include "BpNetwork.hpp"

//robot arm event flag, for camera thread control
namespace robot_arm
{
    enum EVENT_FLAG
    {
        TAKE_ROI,
        MISSION_OK
    };
    enum MOVEMENT_METHOD
    {
        FIX_STEP,
        EVEN_VELOCITY
    };
}

//  This function change the motor pwm values from old values
//to new values by a fixed step Δx, where Δx = 1 by default. 
//It will not return until the new values are reached. Be aware
//that some motors may reach their final values earlier.
//@CAUTION: step should be positive integer!
void fixStepMove(std::vector<int>& newVals,
                 UsbCAN& canDev,
                 int ID);


//  This function moves the arm to `newVals` in even velocity.
//All motor move at the same time and stop together when they
//reach the desired positon. It will not return until the new 
//values are reached. 
//@CAUTION: step should be positive integer!
void evenVelMove(std::vector<int>& newVals,
                 UsbCAN& canDev,
                 int ID);


// Make all motor reset to initial states
void reset2initPos(std::vector<int>& newVals,
                   UsbCAN& canDev,
                   int ID,
                   robot_arm::MOVEMENT_METHOD method = robot_arm::EVEN_VELOCITY);


// The motor values of desired position is calculated
//by the network given.
void move2desiredPos(double x,double y,
                     std::vector<int>& newVals,
                     TfNetwork& network,
                     UsbCAN& canDev,
                     int ID,
                     robot_arm::MOVEMENT_METHOD method = robot_arm::EVEN_VELOCITY);


//  This function detect the rectangle area from (150,180) to 
//targetPos to find whether an obstacle is presented and return
//its height in the img.
double obstacleHeight(cv::Mat depthRaw,
                      float depthScale,
                      cv::Point2f targetPos,
                      int frontPixelOffset);


//  This function retrun the angle between the target and the 
//yOz plane. Motor 1 should turn pass this angle so as to make
//the arms and the target in the same plane.
double motor1moveAngle(cv::Vec3d targetPos);
//
//
// This function return the value that motor1 should move in
//order to reach the position that is above the target. It 
//works based on func `motor1moveAngle`, and its coeffictients
//are calculated by <atlab.
int motor1moveValue(cv::Vec3d targetPos,double upperMmOffset);


//  wait until the image of letter to be detected is ready
void getDetectImg(robot_arm::EVENT_FLAG& flag);


//  Before the arm can grab cubes precisely, it should be calibrated
//using this function. The function calculate the offsets in horizontal
//direction of the front camera and upper camera.
void selfCalibration(ArucoMarker& frontMarker,
                     ArucoMarker& upperMarker,
                     double& frontMmOffset, double& upperMmOffset,
                     int& frontPixelOffset, int& upperPixelOffset,
                     UsbCAN& canDev,
                     int ID);

#endif
