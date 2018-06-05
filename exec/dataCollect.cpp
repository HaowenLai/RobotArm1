/* *************************************************************
 *           collect joints position and pwm duty
 *   This file control motor by giving ramdon pwm duty via CAN.
 * Then the positions of joints can be read by CV method. A set
 * of data is then collected and be written to a file as dataset
 * for deep learning training.
 * @Author : Derek Lai
 * @date   : 2018/5/30
 * @version: v1.1
 * *************************************************************/

#include "ArucoMarker.hpp"
#include "UsbCAN.hpp"

#include <unistd.h>
#include <ctime>
#include <fstream>

using namespace std;
using namespace cv;

#define RANGE_MIN 0
#define RANGE_MAX 150


static void PidControl(vector<int>& now, vector<int>& dst)
{
    const float Kp = 0.5;
    for(size_t i=0;i<dst.size();i++)
    {
        int error = Kp*(dst[i]-now[i]);
        if(error>5)
            error = 5;
        else if(error<-5)
            error = -5;
        now[i] += error;
    }
}

int main()
{
    //file writer
    ofstream fout;
    fout.open("/home/savage/data/roboticArm/data1.txt",ios::out|ios::app);

    //marker define
    const Mat M2_cameraMat = (Mat_<double>(3, 3) 
        << 1208.33,0, 303.71, 0, 1209.325, 246.98, 0, 0, 1);
    const Mat M2_distCoeffs = (Mat_<double>(1, 5)
        << -0.3711,-4.0299, 0, 0,22.9040);
    Mat img;
    ArucoMarker m2Marker(vector<int>({7,8,10}), M2_cameraMat, M2_distCoeffs);
    //
    VideoCapture camera(0);
    namedWindow("M2", WINDOW_AUTOSIZE);
    

    //CAN init
    UsbCAN canII;
    VCI_CAN_OBJ canFrame;
    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"usbCAN init successfully"<<endl;
    }

    //generate random number
    vector<int> DutyNow = {0,80};
    vector<int> DutyDst = {0,80};
    RNG rng(time(NULL));

    while(1)
    {
        DutyDst[0]=rng.uniform(RANGE_MIN,RANGE_MAX);
        DutyDst[1]=rng.uniform(RANGE_MIN,RANGE_MAX);

        //limit the angles
        int sum = 0;
        for(int x:DutyDst)
            sum+=x;
        if(sum>200 || sum<80)
            continue;
        cout<<DutyDst[0]<<" "<<DutyDst[1]<<endl;

        //control until error is small enough
        while(abs(DutyDst[0]-DutyNow[0])>2 ||
              abs(DutyDst[1]-DutyNow[1])>2)
        {
            PidControl(DutyNow,DutyDst);
            generateFrame(canFrame,DutyNow);
            canII.transmit(&canFrame,1);
            cout<<DutyNow[0]<<" "<<DutyNow[1]<<endl;
            
            //read steady-state image,0.5s
            for(int i=0;i<15;i++)
            {
                camera >> img;
                // m2Marker.detect(img);
                //m2Marker.outputOffset(img,Point(30,30));
                imshow("M2", img);
                waitKey(30);
            }

            for(int maxCount=0;maxCount<10;maxCount++)
            {
                camera >> img;
                m2Marker.detect(img);
                m2Marker.outputOffset(img,Point(30,30));
                imshow("M2", img);
                waitKey(30);

                if(m2Marker.offset_tVecs.size() == 3)
                {
                    fout << DutyNow[0] << "," << DutyNow[1];
                    for(auto& offset: m2Marker.offset_tVecs)
                    {
                        fout << ",("
                             << offset[0] << ","
                             << offset[1] << ","
                             << offset[2] << ")";
                    }
                    fout << endl;
                    break;
                }
            }//end while

        }//end while
        //cin.get();
    }

    return 0;
}