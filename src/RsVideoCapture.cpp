/* *********************************************************
 *   this file use opencv to display color and depth images
 * from RealSense.
 * ********************************************************/

#include "RsVideoCapture.hpp"

//-------------------- Definition of class `RsVideoCapture`------------------
//public
RsVideoCapture::RsVideoCapture(const char* serialNumber)
{
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    
    //serial number of the device, on the box `S/N`
    if(serialNumber != nullptr)
        cfg.enable_device(std::string(serialNumber));

    rs2::pipeline_profile profile = pipe.start(cfg);
    depth_scale = get_depth_scale(profile.get_device());
}

RsVideoCapture& RsVideoCapture::operator>>(cv::Mat& img)
{
    getMat(img);
    return *this;
}

//private
inline void RsVideoCapture::getMat(cv::Mat& colorMat)
{
    rs2::frameset data = pipe.wait_for_frames(); 
    auto colorFrame = data.get_color_frame();
    auto depthFrame = data.get_depth_frame();

    // Create color image
	cv::Mat rgb( colorFrame.get_height(),
				 colorFrame.get_width(),
				 CV_8UC3,
				 (uchar *)colorFrame.get_data());
    colorMat = rgb;

    // Create depth mat
	cv::Mat depth16(depthFrame.get_height(),
					depthFrame.get_width(),
					CV_16UC1,
					(uint16_t*)depthFrame.get_data());
    DepthRaw = depth16;
}

inline float RsVideoCapture::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor is a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
//--------------------End definition of class `RsVideoCapture`------------------

void remove_background(cv::Mat& colorImg, RsVideoCapture& camera, float distMin, float distMax)
{
    //const float distMin = 0.5f;
    //const float distMax = 0.7f;

    for (int y = 0; y < 480; y++)
    {
        auto *data = camera.DepthRaw.ptr<uint16_t>(y);  //get y-th row address
        
        for (int x = 0; x < 640; x++)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = camera.depth_scale * data[x];

            // set the pixel out of range to be gray color, which the wanted area is stayed.
            if (pixels_distance < distMin || pixels_distance > distMax)
            {
                // Set pixel to "background" gray color (215,212,213)
                colorImg.at<cv::Vec3b>(y,x) = cv::Vec3b(215,212,213);
            }
        }
    }
}
