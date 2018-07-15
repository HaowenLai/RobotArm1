/* ********************************************
 *   This program can let user manipulate the
 * robotic arm manually. You can refer to the 
 * help message for specific key method.
 * *******************************************/
#include "parameters.hpp"
#include "UsbCAN.hpp"
#include "control.hpp"
#include "ArucoMarker.hpp"
#include "RsVideoCapture.hpp"
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
    using namespace robot_arm::cameraParams;

    ArucoMarker rsMarker(vector<int>({5,6,8}), RS_CM, RS_Dist);
    ArucoMarker m2Marker0(vector<int>({4}), arm_CM, arm_Dist);
    ArucoMarker m2Marker1(vector<int>({4}), upper_CM, upper_Dist);

    VideoCapture camera0(0);    //arm camera
    VideoCapture camera1(1);    //upper camera
    RsVideoCapture camera_rs;
    if(!camera0.isOpened())
    {
        cout<<"cannot open camera"<<endl;
        return -1;
    }

    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_250K))
    {
        cout<<"CAN init successfully"<<endl;
    }

    Mat img0,img1,img2;
    helpMsg();
    namedWindow("front",WINDOW_AUTOSIZE);
    namedWindow("arm",WINDOW_AUTOSIZE);
    namedWindow("upper",WINDOW_AUTOSIZE);
    
    //Arm1 init
    vector<int> newValue1;
    reset2initPos(newValue1,canII,1);

    //control logic variables
    int motorNum = 0;
    int pwmValue = 128;
    
    while(1)
    {
        camera_rs >> img0;
        camera0   >> img1;
        camera1   >> img2;
        rsMarker.detect(img0);
        rsMarker.outputOffset(img0,Point(30,30));
        m2Marker0.detect(img1);
        m2Marker0.outputOffset(img1,Point(30,30));
        m2Marker1.detect(img2);
        m2Marker1.outputOffset(img2,Point(30,30));
        imshow("front",img0);
        imshow("arm",img1);
        imshow("upper",img2);

        // if(m2Marker1.index(4)!=-1)
        // {
        //     auto m1angle = motor1moveAngle(m2Marker1.offset_tVecs[m2Marker1.index(4)]);
        //     newValue1[0] = -90.91 * m1angle + 127;
        //     fixStepMove(newValue1,canII,1);
        //     cout << m1angle << endl;
        // }

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
            fixStepMove(newValue1,canII,1);
            for(int i=0;i<7;i++)
                cout<<newValue1[i] << " ";
            cout<<endl;
            break;
          case '.':
            pwmValue = (pwmValue==0)?0:pwmValue-1;
            newValue1[motorNum] = pwmValue;
            fixStepMove(newValue1,canII,1);
            for(int i=0;i<7;i++)
                cout<<newValue1[i] << " ";
            cout<<endl;
            break;
          case 'r':
            reset2initPos(newValue1,canII,1);
            break;
          
          default:
            break;
        }//end switch

        pwmValue = newValue1[motorNum];
    }//end while



    return 0;
}
