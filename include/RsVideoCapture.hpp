/* ********************************************
 * Head file for rsDev.cpp
 * @description: class to use RS more simply. 
 * @author : Derek Lai
 * @date   : 2018/5/21
 * Copyright(c) All right reserved
 * *******************************************/

#ifndef __RSVIDEOCAPTURE_HPP__
#define __RSVIDEOCAPTURE_HPP__

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>


class RsVideoCapture
{
  public:
    //SerialNumber can be read in the box. If no S/N is passed,
    //it will open a Realsense device randomly
    RsVideoCapture(const char* serialNumber = nullptr);

    //Overload >> , to put a color image to a cv::Mat
    RsVideoCapture& operator>>(cv::Mat& img);
  
  private:
    //Transform RS frame to cv::Mat and get both color image and 
    //depth raw information, though its parameter only contain `colorMat`
    void getMat(cv::Mat& colorMat);

    //Depth scale of each RS device may be different, so we need to get it.
    //Called in the construct function
    float get_depth_scale(rs2::device dev);

  public:
    cv::Mat DepthRaw;
    float depth_scale;  
  private:
    rs2::pipeline pipe;
    
};

//Simply keep the pixel between range (distMin,distMax).
//The result of this func is not satisfying.
void remove_background(cv::Mat& colorImg, RsVideoCapture& camera,
                       float distMin, float distMax);



#endif
