/* *************************************************************
 *           collect joints position and pwm duty
 *   This file control motor by giving ramdon pwm duty via CAN.
 * Then the positions of joints can be read by CV method. A set
 * of data is then collected and be written to a file as dataset
 * for deep learning training.
 * @Author : Derek Lai
 * @date   : 2018/6/13
 * @version: v2.0
 * *************************************************************/

#include "ArucoMarker.hpp"
#include "control.hpp"
#include "UsbCAN.hpp"

#include <unistd.h>
#include <pthread.h>
#include <ctime>
#include <fstream>

using namespace std;
using namespace cv;

#define MOTOR2_MIN 50
#define MOTOR2_MAX 119
#define MOTOR3_MIN 110
#define MOTOR3_MAX 165
#define MOTOR5_MIN 196
#define MOTOR5_MAX 255

bool gDangerFlag = false;
bool gRecordFlag = false;


void* camera_thread(void* data)
{
    //file writer
    ofstream fout;
    fout.open("/home/savage/data/roboticArm/data4.txt",ios::out|ios::app);
    
    //marker define
    const Mat M2_cameraMat = (Mat_<double>(3, 3) 
        << 1208.33,0, 303.71, 0, 1209.325, 246.98, 0, 0, 1);
    const Mat M2_distCoeffs = (Mat_<double>(1, 5)
        << -0.3711,-4.0299, 0, 0,22.9040);
    //
    Mat img;
    ArucoMarker m2Marker(vector<int>({5,8}), M2_cameraMat, M2_distCoeffs);
    VideoCapture camera(0);
    namedWindow("M2", WINDOW_AUTOSIZE);
    
    vector<int>* pwmDuty = (vector<int>*)data;
    
    while(1)
    {
        camera >> img;
        m2Marker.detect(img);
        if(m2Marker.offset_tVecs.size()==2 && m2Marker.offset_tVecs[1][1]>120.0)
        {
            gDangerFlag = true;
            for(int i=0;i<10;i++)
            {
                camera >> img;
                imshow("M2",img);
                waitKey(20);
            }
        }
        imshow("M2",img);
        waitKey(20);

        //try reading coordinates for at most `maxCount`
        for(int maxCount=0;gRecordFlag && maxCount<10;maxCount++)
        {
            camera >> img;
            m2Marker.detect(img);
            m2Marker.outputOffset(img,Point(30,30));
            imshow("M2", img);
            waitKey(20);
            //write data to file
            if(m2Marker.offset_tVecs.size() == 2)
            {
                fout << (*pwmDuty)[1] << "," 
                     << (*pwmDuty)[2] << ","
                     << (*pwmDuty)[4];
                for(auto& offset: m2Marker.offset_tVecs)
                {
                    fout << ","
                        << offset[0] << ","
                        << offset[1] << ","
                        << offset[2] ;
                }
                fout << endl;
                break;
            }
        }//end for
        gRecordFlag = false;
    
    }//end while(1)

    pthread_exit(0);
}


int main()
{
    //CAN init
    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"usbCAN init successfully"<<endl;
    }

    //Arm1 init
    vector<int> oldValue1 {127,250,50,125,235,170,128};
    vector<int> newValue1 {127,250,50,125,235,170,128};
    fixStepMove(oldValue1,newValue1,canII,1);

    //generate random number
    RNG rng(time(NULL));

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread,NULL,camera_thread,&oldValue1);

    cout << "---get ready and press <enter>---" << endl;
    cin.get();
    
    while(1)
    {
        newValue1[1]=rng.uniform(MOTOR2_MIN,MOTOR2_MAX);
        newValue1[2]=rng.uniform(MOTOR3_MIN,MOTOR3_MAX);
        newValue1[4]=rng.uniform(MOTOR5_MIN,MOTOR5_MAX);

        cout<<newValue1[1]<<" "<<newValue1[2]<<" "<<newValue1[4]<<endl;

        //move the arm
        while(!gDangerFlag)
        {
            size_t satisfiedNum = 0;
            for(size_t i=0;i<oldValue1.size();i++)
            {
                if(newValue1[i]-oldValue1[i] >= 1)
                    oldValue1[i] += 1;
                else if(newValue1[i]-oldValue1[i] <= -1)
                    oldValue1[i] += -1;
                else
                    satisfiedNum++;
            }

            VCI_CAN_OBJ can;
            generateFrame(can,oldValue1,1);
            canII.transmit(&can,1);

            if(satisfiedNum == oldValue1.size())
            {
                usleep(500*1000);   //let it calm down
                gRecordFlag = true;
                break;
            }
            usleep(10*1000);
        }//end while(!gDangerFlag), move arm
        
        gDangerFlag = false;
        while(gRecordFlag)
            usleep(10*1000);; //wait until data is recorded
            
    }//end while(1)

    return 0;
}
