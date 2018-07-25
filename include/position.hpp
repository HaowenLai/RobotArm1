/* *************************************************************
 *                 Head file for position.cpp
 *   Class to get position.
 *   There two classes, namely `ArucoMarker` and `CubePosition`.
 * Class `ArucoMarker` deals with the marker positon, whose unit
 * is mm in real world coordinate. Class `CubePosition` to deal 
 * with the position of cubes without markers by finding its
 * contour. Its unit is pixel in image coordinate.
 * 
 * @author : Derek Lai
 * @date   : 2018/7/18
 * @version: v3.2
 * Copyright(c) All right reserved
** ************************************************************/

#ifndef __POSITION_HPP__
#define __POSITION_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoMarker
{
  public:
    ArucoMarker(std::vector<int>&& watchID,
                const cv::Mat cameraMat,
                const cv::Mat distCoeff,
                enum cv::aruco::PREDEFINED_DICTIONARY_NAME dn = cv::aruco::DICT_5X5_50);
    void calibrateOrigin(int calibMarkId);
    void detect(cv::Mat& img);
   
    //return the rotate angle of the marker
    float angle(int id);
    
    //return the first corner of specific index
    cv::Point2f firstCorner(int id);
    
    //  Judge if new frame has come after the last time that
    //this function is called.
    bool isNewFrame();

    //this function return the index of ID in `offfset<>`
    //if such id does NOT exist, it returns -1
    int index(int id);

    void outputOffset(cv::Mat& img,cv::Point&& point); //display on image
    void outputOffset(bool clearConsole = true);       //display on console

    

  private:
    const std::vector<int> watchMarkIds;
    const cv::Mat cameraMatrix;
    const cv::Mat distCoeffs;
    
    cv::Ptr<cv::aruco::Dictionary> dict;
    std::vector<int> markerIds;
    std::vector<int> foundIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    cv::aruco::DetectorParameters parameter;

    std::vector<cv::Vec3d> rVecs;
    std::vector<cv::Vec3d> tVecs;
    cv::Vec3d origin_rVecs;
    cv::Vec3d origin_tVecs;

    int frameStamp;

  public:
    std::vector<cv::Vec3d> offset_rVecs;
    std::vector<cv::Vec3d> offset_tVecs;

};



//To generate ArUco markers, 80*80 pixel, using
//5*5_50 dictionary. It will show the marker and
//wait before the function returns.
void generateMarker(int id);


//  This class deal with the position of cubes without markers.
//It uses binaryzation method and then find contour to locate
//the cube.
class CubePosition
{
  public:
    CubePosition(cv::Rect detectArea,
                 double validLenMax,
                 double validLenMin);
    
    //return the rotate angle of the marker
    float angle();
    
    //this function check if there is a cube in the image
    bool cubeExist();

    //detect cube position from image
    void detect(cv::Mat& img);
   
    //draw the boundry of the detected cube
    void drawBoundry(cv::Mat& img);
    
    //return the first corner of specific index
    cv::Point2f firstCorner();
    
    //get cube position
    cv::Point2f getPosition();
    
    //  Judge if new frame has come after the last time that
    //this function is called.
    bool isNewFrame();

    //|display functions..|
    //display position on image
    void outputPos(cv::Mat& img,cv::Point&& startPoint);
    //display position on console
    void outputPos(bool clearConsole = true);
  
  private:
    const cv::Rect detectArea;       //detect ROI of the img
    const double validPerimeterMax;  //filter invalid contours
    const double validPerimeterMin;  //filter invalid contours

    bool cubeExistFlag;           //true if cube exists in the img
    unsigned int  frameStamp;     //increase 1 when `detect` func is called
    cv::RotatedRect cubeBox;      //the min box enclose the cube


};


//convert upper pixel coordinate to front aruco-marker coordinate
cv::Vec3d upperPC2frontAC(cv::Point2f upperPC);


//convert upper pixel coordinate to upper aruco-marker coordinate
cv::Vec3d upperPC2upperAC(cv::Point2f upperPC);


#endif
