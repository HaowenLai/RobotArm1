#include "parameters.hpp"
#include "RsVideoCapture.hpp"
#include "position.hpp"
#include "control.hpp"
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
    using namespace robot_arm::cameraParams;
    using namespace robot_arm::cubePos;

    Mat img_m0;
    ArucoMarker m2Marker0(vector<int>({4}), upper_CM, upper_Dist);
    CubePosition cubeP(upperArea,upperValidLenMax,upperValidLenMin);

    helpMsg();

    // RsVideoCapture camera_rs;
    VideoCapture camera1(3);
    // VideoCapture camera1(0);
    // camera.set(CV_CAP_PROP_FRAME_WIDTH,1024);
    // camera.set(CV_CAP_PROP_FRAME_HEIGHT,768);


    namedWindow("M0", WINDOW_AUTOSIZE);
    // namedWindow("M1", WINDOW_AUTOSIZE);
    // namedWindow("RS", WINDOW_AUTOSIZE);

    while (1)
    {
        camera1 >> img_m0;
        cubeP.detect(img_m0);
        m2Marker0.detect(img_m0);
        
        cubeP.drawBoundry(img_m0);
        cubeP.outputPos(img_m0,Point(30,30));
        m2Marker0.outputOffset(img_m0,Point2i(30,60));
        
        //cout<<upperPC2upperAC(cubeP.getPosition())<<endl;
        auto targetPC = cubeP.getPosition();
        auto targetUPos = upperPC2upperAC(targetPC);
        cout<< motor1moveValue(targetUPos)<<endl;
  
        imshow("M0", img_m0);
        
        switch ((char)waitKey(50))
        {
          case 'c':
            m2Marker0.calibrateOrigin(5);
            break;
          case 'q':
            return 0;
          case 't':
            take_photo(img_m0);
            break;
          default:
            break;
        }
    }

    return 0;
}
