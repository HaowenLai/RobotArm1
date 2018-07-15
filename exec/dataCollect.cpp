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
#include "RsVideoCapture.hpp"
#include "control.hpp"
#include "UsbCAN.hpp"
#include "parameters.hpp"

#include <unistd.h>
#include <pthread.h>
#include <ctime>
#include <fstream>

using namespace std;
using namespace cv;

#define MOTOR2_MIN 38
#define MOTOR2_MAX 170
#define MOTOR3_MIN 100
#define MOTOR3_MAX 160

bool gRecordFlag = false;


void* camera_thread(void* data)
{
    using namespace robot_arm::cameraParams;
    
    //file writer
    ofstream fout;
    fout.open("/home/savage/data/roboticArm/data9.txt",ios::out|ios::app);
    
    Mat img;
    ArucoMarker m2Marker(vector<int>({5}), RS_CM, RS_Dist);
    RsVideoCapture camera;
    namedWindow("rs", WINDOW_AUTOSIZE);
    
    vector<int>* pwmDuty = (vector<int>*)data;
    
    while(1)
    {
        camera >> img;
        m2Marker.detect(img);
        m2Marker.outputOffset(img,Point(30,30));

        imshow("rs",img);
        waitKey(20);

        //try reading coordinates for at most `maxCount`
        for(int maxCount=0;gRecordFlag && maxCount<10;maxCount++)
        {
            //write data to file
            if(m2Marker.index(5) != -1)
            {
                fout << (*pwmDuty)[1] << "," 
                     << (*pwmDuty)[2];
                
                auto offset = m2Marker.offset_tVecs[m2Marker.index(5)];
                fout << "," << offset[0]
                     << "," << offset[1]
                     << "," << offset[2];
                fout << endl;
                break;
            }
            camera >> img;
            m2Marker.detect(img);
            m2Marker.outputOffset(img,Point(30,30));
            imshow("rs", img);
            waitKey(20);

            if(maxCount==9)
                cout<<"do NOT record\n\n";
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
    vector<int> newValue1;
    reset2initPos(newValue1,canII,1);
    newValue1[4] = 110;
    evenVelMove(newValue1,canII,1);

    //generate random number
    RNG rng(time(NULL));

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread,NULL,camera_thread,&newValue1);

    cout << "---get ready and press <enter>---" << endl;
    cin.get();
    
    while(1)
    {
        newValue1[1]=rng.uniform(MOTOR2_MIN,MOTOR2_MAX);
        newValue1[2]=rng.uniform(MOTOR3_MIN,MOTOR3_MAX);
        cout<<newValue1[1]<<" "<<newValue1[2]<<endl;

        //move the arm
        evenVelMove(newValue1,canII,1);
        usleep(500*1000);   //let it calm down
        gRecordFlag = true;
        
        while(gRecordFlag)
            usleep(10*1000); //wait until data is recorded
    }//end while(1)

    return 0;
}
