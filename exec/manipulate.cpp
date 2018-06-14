/* ********************************************
 *   This program can let user manipulate the
 * robotic arm manually. You can refer to the 
 * help message for specific key method.
 * *******************************************/

#include "UsbCAN.hpp"
#include "control.hpp"
#include "ArucoMarker.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

//help message
static inline void helpMsg()
{
    cout << "help messages for this program\n"
            "key '1' : select motor 1\n"
            "key '2' : select motor 2\n"
            "key '3' : select motor 3\n"
            "key '4' : select motor 4\n"
            "key '5' : select motor 5\n"
            "key '6' : select motor 6\n"
            "key '7' : select motor 7\n"
            "key ',' : increase value\n"
            "key '.' : decrease value\n"
            "---press <Enter> to continue---";
    cin.get();
}

int main()
{
    //camera instrinc matrix and distort coefficients
    const Mat M2_cameraMatrix = (Mat_<double>(3, 3) 
        << 1208.33,0, 303.71, 0, 1209.325, 246.98, 0, 0, 1);
    const Mat M2_distCoeffs = (Mat_<double>(1, 5)
        << -0.3711,-4.0299, 0, 0,22.9040);
    ArucoMarker m2Marker(vector<int>({5,8}), M2_cameraMatrix, M2_distCoeffs);
    

    VideoCapture camera(0);
    if(!camera.isOpened())
    {
        cout<<"cannot open camera"<<endl;
        return -1;
    }

    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"CAN init successfully"<<endl;
    }

    Mat img;
    helpMsg();
    namedWindow("viewer",WINDOW_AUTOSIZE);
    
    //Arm1 init
    vector<int> oldValue1 {127,250,50,125,235,170,128};
    vector<int> newValue1 {127,250,50,125,235,170,128};
    fixStepMove(oldValue1,newValue1,canII,1);

    //control logic variables
    int motorNum = 0;
    int pwmValue = 128;
    
    while(1)
    {
        camera >> img;
        m2Marker.detect(img);
        m2Marker.outputOffset(img,Point(30,30));
        imshow("viewer",img);
        
        switch ((char)waitKey(30))
        {
          case '1':
            motorNum = 0;break;
          case '2':
            motorNum = 1;break;
          case '3':
            motorNum = 2;break;
          case '4':
            motorNum = 3;break;
          case '5':
            motorNum = 4;break;
          case '6':
            motorNum = 5;break;
          case '7':
            motorNum = 6;break;
          case ',':
            pwmValue = (pwmValue==255)?255:pwmValue+1;
            newValue1[motorNum] = pwmValue;
            fixStepMove(oldValue1,newValue1,canII,1);
            for(int i=0;i<7;i++)
                cout<<oldValue1[i] << " ";
            cout<<endl;
            break;
          case '.':
            pwmValue = (pwmValue==0)?0:pwmValue-1;
            newValue1[motorNum] = pwmValue;
            fixStepMove(oldValue1,newValue1,canII,1);
            for(int i=0;i<7;i++)
                cout<<oldValue1[i] << " ";
            cout<<endl;
            break;
          default:
            break;
        }//end switch

        pwmValue = oldValue1[motorNum];
    }//end while



    return 0;
}
