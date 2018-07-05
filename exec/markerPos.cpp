#include "RsVideoCapture.hpp"
#include "ArucoMarker.hpp"
#include <time.h>

using namespace cv;
using namespace std;

//help message
static inline void helpMsg()
{
    cout << "help messages for this program\n"
            "key 'c' : to calibrate all markers original positions\n"
            "key 't' : take photo\n"
            "key 'q' : quit the program\n"
            "\n---press <Enter> to continue---";
    cin.get();
    printf("\n\n\n\n\n");
}

void take_photo(Mat& img)
{
	const string photo_path = "../data/";
    // get system time as file name..
	time_t tt;
	time(&tt);
	tt += 8*3600;   //transform the time zone
	tm *t = gmtime(&tt);
	char timeBuff[30];
	sprintf(timeBuff,"%d-%02d-%02d %02d:%02d:%02d",
	        t->tm_year +1900,
	        t->tm_mon+1,
	        t->tm_mday,
	        t->tm_hour,
	        t->tm_min,
	        t->tm_sec);
	
	string img_path = photo_path + timeBuff + ".jpg";
	imwrite(img_path,img);
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




//detect ArUco markers and estimate pose
int main()
{
    //camera instrinc matrix and distort coefficients
    const Mat M2_cameraMatrix0 = (Mat_<double>(3, 3) 
        << 1208.33,0, 303.71, 0, 1209.325, 246.98, 0, 0, 1);
    const Mat M2_distCoeffs0 = (Mat_<double>(1, 5)
        << -0.3711,-4.0299, 0, 0,22.9040);
    const Mat M2_cameraMatrix1 = (Mat_<double>(3, 3) 
        << 926.64,0, 327.76, 0, 926.499, 246.74, 0, 0, 1);
    const Mat M2_distCoeffs1 = (Mat_<double>(1, 5)
        << -0.4176,0.1635, 0, 0);
    const Mat RS_cameraMatrix = (Mat_<double>(3, 3)
        << 622.60,0, 312.12, 0, 623.37, 235.86, 0, 0, 1);
    const Mat RS_distCoeffs = (Mat_<double>(1, 4)
        << 0.156,-0.2792, 0, 0);

    Mat img_m0,img_m1;
    ArucoMarker m2Marker0(vector<int>({5,6}), RS_cameraMatrix, RS_distCoeffs);
    // ArucoMarker m2Marker1(vector<int>({4}), M2_cameraMatrix1, M2_distCoeffs1);

    helpMsg();

    // VideoCapture camera1(4);
    // VideoCapture camera1(0);
    // camera.set(CV_CAP_PROP_FRAME_WIDTH,1024);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT,768);

    RsVideoCapture camera_rs;

    namedWindow("M0", WINDOW_AUTOSIZE);
    // namedWindow("M1", WINDOW_AUTOSIZE);
    // namedWindow("RS", WINDOW_AUTOSIZE);

    while (1)
    {
        camera_rs >> img_m0;
        m2Marker0.detect(img_m0);
        m2Marker0.outputOffset(img_m0,Point(30,30));
        
        // camera1 >> img_m1;
        // m2Marker1.detect(img_m1);
        // m2Marker1.outputOffset(img_m1,Point(30,30));

        imshow("M0", img_m0);
        // imshow("M1", img_m1);
        
        //obstacle test
        // if(m2Marker0.index(6)!=-1)
        // {
        //     auto targetPos = m2Marker0.firstCorner(6);
        //     auto height = obstacleHeight(camera_rs.DepthRaw,
        //         camera_rs.depth_scale,targetPos);
        //     cout << height <<endl;
        // }
        
        
        switch ((char)waitKey(50))
        {
          case 'c':
            m2Marker0.calibrateOrigin(5);
            break;
          case 'q':
            return 0;
          case 't':
            take_photo(img_m1);
            break;
          default:
            break;
        }
    }

    return 0;
}
