/* ***********************************************************
 * Head file for ArucoMarker.cpp
 * @description: class to track markers position more easily. 
 * @author : Derek Lai
 * @date   : 2018/6/1
 * @version: v1.5
 * Copyright(c) All right reserved
** ************************************************************/

#ifndef __ARUCOMARKER_HPP__
#define __ARUCOMARKER_HPP__

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
   
    void outputOffset(cv::Mat& img,cv::Point&& point); //display on image
    void outputOffset(bool clearConsole = true);      //display on console

    

  private:
    const std::vector<int> watchMarkIds;
    const cv::Mat cameraMatrix;
    const cv::Mat distCoeffs;
    // int specificMarkIndex;
    
    cv::Ptr<cv::aruco::Dictionary> dict;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<std::vector<cv::Point2f>> rejectedCandidates;
    cv::aruco::DetectorParameters parameter;

    std::vector<cv::Vec3d> rVecs;
    std::vector<cv::Vec3d> tVecs;
    cv::Vec3d origin_rVecs;
    cv::Vec3d origin_tVecs;

  public:
    std::vector<cv::Vec3d> offset_rVecs;
    std::vector<cv::Vec3d> offset_tVecs;

};



//To generate ArUco markers, 80*80 pixel, using
//5*5_50 dictionary. It will show the marker and
//wait before the function returns.
void generateMarker(int id);

#endif
