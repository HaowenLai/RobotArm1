/* ***********************************************************
 *   This file contains parameters that this project will use
 * @Author : Derek Lai
 * @Date   : 2018/7/9
 * @Version: v1.0
 * Copyright(c) All right reserved
 * ***********************************************************/

#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__

#include <opencv2/opencv.hpp>

namespace robot_arm
{
    namespace cameraParams
    {
        //front camera, realsense, S/N: 747612060738
        const cv::Mat RS_CM = (cv::Mat_<double>(3, 3)
            << 600.628,0, 309.183, 0, 601.173, 238.65, 0, 0, 1);
        const cv::Mat RS_Dist = (cv::Mat_<double>(1, 4)
            << 0.0866,-0.1543, 0, 0);
        
        //upper camera, 2.0M pixels, 1/2.7'', 6mm
        const cv::Mat upper_CM = (cv::Mat_<double>(3, 3)
            << 921.826,0, 314.833, 0, 922.173, 249.803, 0, 0, 1);
        const cv::Mat upper_Dist = (cv::Mat_<double>(1, 4)
            << -0.4944, 0.4279, 0, 0);

        //arm camera, 4mm
        const cv::Mat arm_CM = (cv::Mat_<double>(3, 3)
            << 495.0 ,0, 351.634, 0, 498.223, 232.914, 0, 0, 1);
        const cv::Mat arm_Dist = (cv::Mat_<double>(1, 4)
            << 0.1084, -0.1640, 0, 0);
    };















};

#endif
