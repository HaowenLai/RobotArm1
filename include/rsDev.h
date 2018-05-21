/* ********************************************
 * Head file for rsDev.cpp
 * @description: class to use RS more simply. 
 * @author : Derek Lai
 * @date   : 2018/5/21
 * Copyright(c) All right reserved
 * *******************************************/

#ifndef __RSDEV_H__
#define __RSDEV_H__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>


class RsVideoCapture
{
  public:
    RsVideoCapture(const char* serialNumber = nullptr);
    RsVideoCapture& operator>>(cv::Mat& img);
  private:
    void getMat(cv::Mat& colorMat);
    float get_depth_scale(rs2::device dev);

  public:
    cv::Mat DepthRaw;
    float depth_scale;  
  private:
    rs2::pipeline pipe;
    
};

void remove_background(cv::Mat& colorImg, RsVideoCapture& camera,
                       float distMin, float distMax);



#endif