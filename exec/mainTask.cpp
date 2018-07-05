/* *************************************************************
 *          Main task that the robotic arm performs
 *   The robotic arm will locate the target in three-dimensions.
 * Then, it will move to the place roughly and adjust precisely
 * by the feedback from camera attached to its clip. After that,
 * it will turn the bottom surface of the target and let it face
 * the front camera, and the program detects which letter it is.
 * Finally, judged by different detections, the arm moves the
 * target to corresponding places.
 *
 * @Author : Derek Lai (LHW)
 * @Date   : 2018/7/5
 * ************************************************************/

#include "UsbCAN.hpp"
#include "BpNetwork.hpp"
#include "ArucoMarker.hpp"
#include "RsVideoCapture.hpp"
#include "Wifi.hpp"
#include "control.hpp"

#include <pthread.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

static void* camera_thread(void* data);


//camera instrinc matrix and distort coefficients
const Mat RS_cameraMatrix = (Mat_<double>(3, 3)
    << 622.60,0, 312.12, 0, 623.37, 235.86, 0, 0, 1);
const Mat RS_distCoeffs = (Mat_<double>(1, 4)
    << 0.156,-0.2792, 0, 0);
const Mat M2_cameraMatrix1 = (Mat_<double>(3, 3) 
    << 926.64,0, 327.76, 0, 926.499, 246.74, 0, 0, 1);
const Mat M2_distCoeffs1 = (Mat_<double>(1, 5)
    << -0.4176,0.1635, 0, 0);
ArucoMarker m2Marker0(vector<int>({5,6,8}), RS_cameraMatrix, RS_distCoeffs);
ArucoMarker m2Marker1(vector<int>({4}), M2_cameraMatrix1, M2_distCoeffs1);
ArucoMarker m2Marker2(vector<int>({4}), M2_cameraMatrix1, M2_distCoeffs1);
RsVideoCapture camera_rs;

//global variables, to communicate with camera thread
auto controlFlag = robot_arm::MISSION_OK;
Mat detectLetterImg;

// #define _DEBUG_MODE_

int main()
{
    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"init successfully\n"
            <<"transmitting..."<<endl;
    }

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread, NULL, camera_thread, NULL);

    //! Network parameter
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "tf_network";
    string funcName   = "bp_main";
    TfNetwork network(modulePath,moduleName,funcName);
    
    //wifi communication
    Wifi s_wifi(1234,1);

    //Arm1 initialize
    vector<int> newValue1;
    reset2initPos(newValue1,canII,1);
    cout << "Program is ready.\nPress <enter> to continue.." << endl;
    cin.get();
    //------------------ begin algrithm -----------------------

    const int diff5_8 = 92; //height difference between #5 and #8
    
    //get target position
    Vec3d targetPos;
    double obstacleH;
    double motor1angle;
    cout<<"start to get target position...\n";
    while(true)
    {
        if(m2Marker0.index(6) != -1 && m2Marker2.index(4)!=-1)
        {
            targetPos = m2Marker0.offset_tVecs[m2Marker0.index(6)];
            obstacleH = obstacleHeight(camera_rs.DepthRaw,
                camera_rs.depth_scale,
                m2Marker0.firstCorner(6));
            motor1angle = motor1moveAngle(m2Marker2.offset_tVecs[m2Marker2.index(4)]);
            
            cout<<"target position got ~\n";
            break;
        }
    }

    //deal with obstacle
    if(obstacleH < 50)
    {
        newValue1[1] = 230;
        fixStepMove(newValue1,canII,1);
        
        move2desiredPos(targetPos[0],obstacleH-diff5_8-35,
                    newValue1,network,canII,1);
    }
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif

    //above the target, for more precise adjustment
    int x_offset = 5;
    move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-25,
                    newValue1,network,canII,1);
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif

    //motor #1 roughly move (rotate)
    newValue1[0] = -90.91 * motor1angle + 127;
    fixStepMove(newValue1,canII,1);
    cout << motor1angle << endl;
    
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif
    
    //adjust motor #6 (rotate)
    while(true)
    {
        const float epsilon = 0.02f;

        if(!m2Marker1.isNewFrame())
            continue;

        if(m2Marker1.index(4) != -1)
        {
            auto angle_offset = m2Marker1.angle(4);
            
            if(angle_offset > epsilon)
            {
                newValue1[5] -= 1;
                fixStepMove(newValue1,canII,1);
            }
            else if(angle_offset < -epsilon)
            {
                newValue1[5] += 1;
                fixStepMove(newValue1,canII,1);
            }
            else
            {
                cout<<"#6 is in position ~\n";
                break;
            }
        }//end if(m2Marker1.index(4) != -1)
        else
        {
            //ensure upper label is in vision
            x_offset++;
            move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-25,
                    newValue1,network,canII,1);
            usleep(20*1000);
        }
    }
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif

    //adjust motor #4(prepare for dig-into step)
    while(true)
    {
        const float epsilon = 0.5f;
        const float centerX = 8.5f;

        if(!m2Marker1.isNewFrame())
            continue;

        if(m2Marker1.index(4) != -1)
        {
            float targetX = m2Marker1.offset_tVecs[m2Marker1.index(4)][0];
            
            if(targetX - centerX > epsilon)
            {
                newValue1[3] -= 1;
                fixStepMove(newValue1,canII,1);
            }
            else if(targetX - centerX < -epsilon)
            {
                newValue1[3] += 1;
                fixStepMove(newValue1,canII,1);
            }
            else
            {
                cout<<"#4 is in position ~\n";
                break;
            }
        }//end if(m2Marker1.index(4) != -1)
    }
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif


    //dig into the target
    move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-5,
                    newValue1,network,canII,1);
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif

    //adjust motor #5 to vertical direction
    cout<<"start to adjust #8...\n";
    while(true)
    {
        const double epsilon = 4;

        if(!m2Marker0.isNewFrame())
            continue;

        auto index8 = m2Marker0.index(8);
        if(index8 != -1)
        {
            if(m2Marker0.offset_tVecs[index8][0] - targetPos[0] > epsilon)
            {
                newValue1[4] += 1;
                fixStepMove(newValue1,canII,1);
            }
            else if(m2Marker0.offset_tVecs[index8][0] - targetPos[0] < -epsilon)
            {
                newValue1[4] -= 1;
                fixStepMove(newValue1,canII,1);
            }
            else
            {
                cout<<"#5 is in position ~\n";
                break;
            }
        }//end if(index8 != -1)

        if(newValue1[4]==255 || newValue1[4]==0)
            break;
    }
    #ifdef _DEBUG_MODE_
    cin.get();
    #endif

    //use #7 to grab the cube
    cout<<"start to grab the cube\n";
    newValue1[6] = 140;
    fixStepMove(newValue1,canII,1);
    cout<<"grab the cube successfully ~\n";

    
    //check the bottom surface
    newValue1 = vector<int> {90,110,123,0,235,191,140};
    fixStepMove(newValue1,canII,1);
    getDetectImg(controlFlag);
    s_wifi.sendMsg(detectLetterImg.data,50*50*3,0);
    
    
    //move the cube to another place (92,6)
    move2desiredPos(targetPos[0],-1.0-diff5_8,
                    newValue1,network,canII,1);
    newValue1[0] = 127;
    newValue1[3] = 125;
    newValue1[4] = 255;
    newValue1[5] = 165;
    fixStepMove(newValue1,canII,1);
    move2desiredPos(92.0,6.0-diff5_8,
                    newValue1,network,canII,1);
    cout<<"finish moving the cube ~\n";
    
    //loose the clip
    newValue1[6] = 100;
    fixStepMove(newValue1,canII,1);
    
    //return to initial position
    reset2initPos(newValue1,canII,1);
    cout<<"\n~~~ I FINISH MY JOB ~~~\n-- press 'q' to exit --\n";

    //wait...
    pthread_join(cameraThread, NULL);
    
    return 0;
}



//camera thread for func `moveInRoute`
static void* camera_thread(void* data)
{
    Mat img0, img1, img2;
    VideoCapture camera1(1);    //arm camera
    VideoCapture camera2(0);    //upper camera
    namedWindow("view_front", WINDOW_AUTOSIZE);
    namedWindow("view_arm", WINDOW_AUTOSIZE);
    namedWindow("view_up", WINDOW_AUTOSIZE);
    
    while((char)waitKey(20) != 'q')
    {
        camera_rs >> img0;
        camera1 >> img1;
        camera2 >> img2;
        m2Marker0.detect(img0);
        m2Marker0.outputOffset(img0,Point(30,30));
        m2Marker1.detect(img1);
        m2Marker1.outputOffset(img1,Point(30,30));
        m2Marker2.detect(img2);
        m2Marker2.outputOffset(img2,Point(30,30));
        imshow("view_front",img0);
        imshow("view_arm",img1);
        imshow("view_up",img2);

        if(controlFlag == robot_arm::TAKE_ROI)
        {
            detectLetterImg = img0(Rect(256,144,50,50)).clone();
            imshow("haha",detectLetterImg);
            controlFlag = robot_arm::MISSION_OK;
        }
    }

    pthread_exit(0);
}
