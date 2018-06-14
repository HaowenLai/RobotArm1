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


//detect ArUco markers and estimate pose
int main()
{
    //camera instrinc matrix and distort coefficients
    const Mat M2_cameraMatrix = (Mat_<double>(3, 3) 
        << 1208.33,0, 303.71, 0, 1209.325, 246.98, 0, 0, 1);
    const Mat M2_distCoeffs = (Mat_<double>(1, 5)
        << -0.3711,-4.0299, 0, 0,22.9040);
    // const Mat RS_cameraMatrix = (Mat_<double>(3, 3)
    //     << 622.60,0, 312.12, 0, 623.37, 235.86, 0, 0, 1);
    // const Mat RS_distCoeffs = (Mat_<double>(1, 4)
    //     << 0.156,-0.2792, 0, 0);

    Mat img_m2;
    ArucoMarker m2Marker(vector<int>({5,6,8}), M2_cameraMatrix, M2_distCoeffs);

    helpMsg();

    VideoCapture camera(0);
    // camera.set(CV_CAP_PROP_FRAME_WIDTH,1024);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT,768);

    //RsVideoCapture camera_rs;

    namedWindow("M2", WINDOW_AUTOSIZE);
    // namedWindow("RS", WINDOW_AUTOSIZE);

    while (1)
    {
        camera >> img_m2;
        //camera_rs >> img_rs;
        m2Marker.detect(img_m2);
        m2Marker.outputOffset(img_m2,Point(30,30));
        
        imshow("M2", img_m2);

        switch ((char)waitKey(50))
        {
          case 'c':
            m2Marker.calibrateOrigin(7);
            break;
          case 'q':
            return 0;
          case 't':
            take_photo(img_m2);
            break;
          default:
            break;
        }
    }

    return 0;
}
