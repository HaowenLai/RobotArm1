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

#include "parameters.hpp"
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

// #define _SINGLE_MOVE_MODE_
// #define _NO_VIDEO_MODE_

static void* camera_thread(void* data);

using namespace robot_arm::cameraParams;
ArucoMarker m2Marker0(vector<int>({5,6,8}), RS_CM, RS_Dist);    //rs
ArucoMarker m2Marker1(vector<int>({4}), arm_CM, arm_Dist);      //arm
ArucoMarker m2Marker2(vector<int>({4}), upper_CM, upper_Dist);  //up
RsVideoCapture camera_rs;

//global variables, to communicate with camera thread
auto controlFlag = robot_arm::MISSION_OK;
Mat detectLetterImg;


int main(int argc, char* argv[])
{
    //  The first argument is program path, the second argument
    //is the number of times that this program will run. i.e.
    //how many times will the arm grab the cube.
    if(argc != 2)
    {
        cout << "argument number is wrong!\n"
             << "You must add the times that the arm will grab the cube\n";
        exit(-1);
    }
    int runCount = 1;
    const int runTimes = atoi(argv[1]);

    
    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_250K))
    {
        cout<<"init successfully\n"
            <<"transmitting..."<<endl;
    }

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread, NULL, camera_thread, NULL);

    //! Network parameter
    char programName[100];
    getcwd(programName,100);    //get program path
    string modulePath(programName);
    modulePath = modulePath.substr(0,modulePath.find_last_of('/'))+"/src";
    string moduleName = "tf_network";
    string funcName   = "bp_main";
    TfNetwork network(modulePath,moduleName,funcName);
    
    //wifi communication
    Wifi s_wifi(1234,2);

    //Arm1 initialize
    vector<int> motorValues;
    reset2initPos(motorValues,canII,1);
    cout << "Program is ready.\nPress <enter> to continue.." << endl;
    cin.get();
    //------------------ begin algrithm -----------------------

    const int diff5_8 = 90; //height difference between #5 and #8
    
again:
    //get target position
    Vec3d targetPos;
    double obstacleH;
    int motor1Val;
    cout<<"start to get target position...\n";
    while(true)
    {
        if(m2Marker0.index(6) != -1 && m2Marker2.index(4)!=-1)
        {
            targetPos = m2Marker0.offset_tVecs[m2Marker0.index(6)];
            obstacleH = obstacleHeight(camera_rs.DepthRaw,
                camera_rs.depth_scale,
                m2Marker0.firstCorner(6));
            motor1Val = motor1moveValue(m2Marker2.offset_tVecs[m2Marker2.index(4)]);
            
            cout<<"target position got ~\n";
            break;
        }
    }

    //deal with obstacle
    if(obstacleH < 40)
    {
        cout<<"avoid obstacle"<<endl;
        motorValues[1] = 230;
        fixStepMove(motorValues,canII,1);
        
        move2desiredPos(targetPos[0],obstacleH-diff5_8-35,
                    motorValues,network,canII,1);
    }
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    //above the target, for more precise adjustment
    int x_offset = 5;
    move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-35,
                    motorValues,network,canII,1);
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    //motor #1 roughly move (rotate)
    //motorValues[0] = -94.0 * motor1angle + 126;
    motorValues[0] = motor1Val;
    fixStepMove(motorValues,canII,1);
    cout << motor1Val << endl;
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif
    
    //adjust motor #6 (rotate)
    int marker4AdjustCount = 0;
    while(true)
    {
        const float epsilon = 0.02f;

        if(!m2Marker1.isNewFrame())
            continue;

        if(m2Marker1.index(4) != -1)
        {
            auto angle_offset = m2Marker1.angle(4);
            marker4AdjustCount = 0;
            
            if(angle_offset > epsilon)
            {
                motorValues[5] -= 1;
                fixStepMove(motorValues,canII,1);
            }
            else if(angle_offset < -epsilon)
            {
                motorValues[5] += 1;
                fixStepMove(motorValues,canII,1);
            }
            else
            {
                cout<<"#6 is in position ~\n";
                break;
            }
        }//end if(m2Marker1.index(4) != -1)
        else if(marker4AdjustCount > 10)
        {
            //ensure upper label is in vision
            x_offset++;
            move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-35,
                    motorValues,network,canII,1);
            usleep(20*1000);
        }
        else
        {
            marker4AdjustCount++;
        }
    }
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    //adjust motor #4(prepare for dig-into step)
    while(true)
    {
        const float epsilon = 0.3f;
        const float centerX = 5.8f;

        if(!m2Marker1.isNewFrame())
            continue;

        if(m2Marker1.index(4) != -1)
        {
            float targetX = m2Marker1.offset_tVecs[m2Marker1.index(4)][0];
            
            if(targetX - centerX > epsilon)
            {
                motorValues[3] -= 1;
                fixStepMove(motorValues,canII,1);
            }
            else if(targetX - centerX < -epsilon)
            {
                motorValues[3] += 1;
                fixStepMove(motorValues,canII,1);
            }
            else
            {
                cout<<"#4 is in position ~\n";
                break;
            }
        }//end if(m2Marker1.index(4) != -1)
        usleep(100*1000);
    }
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif


    //dig into the target
    move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-7,
                    motorValues,network,canII,1);
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    //adjust motor #5 to vertical direction
    cout<<"start to adjust motor #5...\n";
    while(true)
    {
        const double epsilon = 16;

        if(!m2Marker0.isNewFrame())
            continue;

        auto index8 = m2Marker0.index(8);
        if(index8 != -1)
        {
            if(m2Marker0.offset_tVecs[index8][0] - targetPos[0] > epsilon)
            {
                motorValues[4] += 1;
                fixStepMove(motorValues,canII,1);
            }
            else if(m2Marker0.offset_tVecs[index8][0] - targetPos[0] < -epsilon)
            {
                motorValues[4] -= 1;
                fixStepMove(motorValues,canII,1);
            }
            else
            {
                cout<<"#5 is in position ~\n";
                break;
            }
        }//end if(index8 != -1)

        if(motorValues[4]==255 || motorValues[4]==0)
            break;
    }
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    //use #7 to grab the cube
    cout<<"start to grab the cube\n";
    motorValues[6] = 140;
    fixStepMove(motorValues,canII,1);
    cout<<"grab the cube successfully ~\n";

    
    //check the bottom surface
    s_wifi.sendMsg(Wifi::MSG_PREPARE_TO_DETECT,0);
    motorValues = vector<int> {79,110,123,0,235,194,140};
    evenVelMove(motorValues,canII,1);
    getDetectImg(controlFlag);
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif
    s_wifi.sendMsg(detectLetterImg.data,50*50*3,0);

    //read the surface information
    while(!s_wifi.recvNewMSG(0));
    if(s_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
    {
        motorValues[0] = 154;
        motorValues[3] = 125;
        motorValues[4] = 255;
        motorValues[5] = 165;
        evenVelMove(motorValues,canII,1);   
        
        motorValues[1] = 62;
        motorValues[2] = 89;
        evenVelMove(motorValues,canII,1);

        //loose clip
        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        
        //reset
        reset2initPos(motorValues,canII,1);
    }
    else
    {
        //move the cube to another place (167,-8.0)
        move2desiredPos(targetPos[0],-20.0-diff5_8,
                        motorValues,network,canII,1);
        motorValues[0] = 127;
        motorValues[3] = 125;
        motorValues[4] = 255;
        motorValues[5] = 165;
        evenVelMove(motorValues,canII,1);
        move2desiredPos(175,-18.0-diff5_8,
                        motorValues,network,canII,1);
        cout<<"finish moving the cube ~\n";
        
        //loose the clip
        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        
        //wait for arm0 to finish stamping
        move2desiredPos(60,-160,motorValues,network,canII,1);
        s_wifi.sendMsg(Wifi::MSG_TARGET_IN_POSITION,1);
        while(!s_wifi.recvNewMSG(1));

        //move the stamped cube to "correct" place
        //get target position
        cout<<"start to get stamped target position\n";
        while(true)
        {
            if(m2Marker0.index(6) != -1 && m2Marker2.index(4)!=-1)
            {
                targetPos = m2Marker0.offset_tVecs[m2Marker0.index(6)];
                motor1Val = motor1moveValue(m2Marker2.offset_tVecs[m2Marker2.index(4)]);
                
                cout<<"target position got ~\n";
                break;
            }
        }
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif

        //above the target, for more precise adjustment
        x_offset = 0;
        move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-35,
                        motorValues,network,canII,1);
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif

        //motor #1 roughly move (rotate)
        //motorValues[0] = -94.0 * motor1angle + 126;
        motorValues[0] = motor1Val;
        fixStepMove(motorValues,canII,1);
        cout << motor1Val << endl;
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif
    
        //adjust motor #6 (rotate)
        motorValues[4] = 246;
        fixStepMove(motorValues,canII,1);
        marker4AdjustCount = 0;
        while(true)
        {
            const float epsilon = 0.02f;

            if(!m2Marker1.isNewFrame())
                continue;

            if(m2Marker1.index(4) != -1)
            {
                auto angle_offset = m2Marker1.angle(4);
                marker4AdjustCount = 0;
                
                if(angle_offset > epsilon)
                {
                    motorValues[5] -= 1;
                    fixStepMove(motorValues,canII,1);
                }
                else if(angle_offset < -epsilon)
                {
                    motorValues[5] += 1;
                    fixStepMove(motorValues,canII,1);
                }
                else
                {
                    cout<<"#6 is in position ~\n";
                    break;
                }
            }//end if(m2Marker1.index(4) != -1)
            else if(marker4AdjustCount > 10)
            {
                //ensure upper label is in vision
                x_offset++;
                move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-35,
                        motorValues,network,canII,1);
                usleep(20*1000);
            }
            else
            {
                marker4AdjustCount++;
            }
        }
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif

        //adjust motor #4(prepare for dig-into step)
        while(true)
        {
            const float epsilon = 0.3f;
            const float centerX = 5.8f;

            if(!m2Marker1.isNewFrame())
                continue;

            if(m2Marker1.index(4) != -1)
            {
                float targetX = m2Marker1.offset_tVecs[m2Marker1.index(4)][0];
                
                if(targetX - centerX > epsilon)
                {
                    motorValues[3] -= 1;
                    fixStepMove(motorValues,canII,1);
                }
                else if(targetX - centerX < -epsilon)
                {
                    motorValues[3] += 1;
                    fixStepMove(motorValues,canII,1);
                }
                else
                {
                    cout<<"#4 is in position ~\n";
                    break;
                }
            }//end if(m2Marker1.index(4) != -1)
            usleep(100*1000);
        }
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif


        //dig into the target
        move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-7,
                        motorValues,network,canII,1);
        usleep(500*1000);
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif

        /*
        //adjust motor #5 to vertical direction
        cout<<"start to adjust motor #5...\n";
        while(true)
        {
            const double epsilon = 16;

            if(!m2Marker0.isNewFrame())
                continue;

            auto index8 = m2Marker0.index(8);
            if(index8 != -1)
            {
                if(m2Marker0.offset_tVecs[index8][0] - targetPos[0] > epsilon)
                {
                    motorValues[4] += 1;
                    fixStepMove(motorValues,canII,1);
                }
                else if(m2Marker0.offset_tVecs[index8][0] - targetPos[0] < -epsilon)
                {
                    motorValues[4] -= 1;
                    fixStepMove(motorValues,canII,1);
                }
                else
                {
                    cout<<"#5 is in position ~\n";
                    break;
                }
            }//end if(index8 != -1)

            if(motorValues[4]==255 || motorValues[4]==0)
                break;
        }
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif
        */

        //use #7 to grab the cube
        cout<<"start to grab the cube\n";
        motorValues[6] = 140;
        fixStepMove(motorValues,canII,1);
        cout<<"grab the cube successfully ~\n";

        //move to "correct" place
        move2desiredPos(targetPos[0]+x_offset,targetPos[1]-diff5_8-45,
                        motorValues,network,canII,1);
        motorValues = vector<int> {94,68,92,125,255,165,140};
        evenVelMove(motorValues,canII,1);

        //loose clip
        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        
        //reset
        reset2initPos(motorValues,canII,1);
    }
    
    //whether it has finished all jobs
    if(runCount < runTimes)
    {
        runCount++;
        goto again;
    }

    //wait...
    cout<<"\n~~~ I FINISH MY JOB ~~~\n-- press 'q' to exit --\n";
    s_wifi.sendMsg(Wifi::MSG_FINISH_ALL_JOB,0);
    s_wifi.sendMsg(Wifi::MSG_FINISH_ALL_JOB,1);
    pthread_join(cameraThread, NULL);
    
    return 0;
}



//camera thread for func `moveInRoute`
static void* camera_thread(void* data)
{
    //white area
    Rect unQualifyArea(Point(174,34),Point(269,120));
    Rect qualifyArea(Point(169,346),Point(264,438));
    
    Mat img0, img1, img2, img_21;
    VideoCapture camera1(0);    //arm camera
    VideoCapture camera2(1);    //upper camera

    #ifndef _NO_VIDEO_MODE_
    namedWindow("view_front", WINDOW_AUTOSIZE);
    namedWindow("view_arm", WINDOW_AUTOSIZE);
    namedWindow("view_up", WINDOW_AUTOSIZE);
    #endif

    while((char)waitKey(20) != 'q')
    {
        camera_rs >> img0;
        camera1 >> img1;
        camera2 >> img2;
        
        //deal with the already grab cubes
        img_21 = img2.clone();
        img2(unQualifyArea).setTo(Scalar(255,255,255));
        img2(qualifyArea).setTo(Scalar(255,255,255));
        
        m2Marker0.detect(img0);
        m2Marker1.detect(img1);
        m2Marker2.detect(img2);

        #ifndef _NO_VIDEO_MODE_
        m2Marker0.outputOffset(img0,Point(30,30));
        m2Marker1.outputOffset(img1,Point(30,30));
        m2Marker2.outputOffset(img2,Point(30,30));
        imshow("view_front",img0);
        imshow("view_arm",img1);
        imshow("view_up",img_21);
        #endif

        if(controlFlag == robot_arm::TAKE_ROI)
        {
            detectLetterImg = img0(Rect(405,118,50,50)).clone();
            imshow("Letter",detectLetterImg);
            controlFlag = robot_arm::MISSION_OK;
        }
    }

    pthread_exit(0);
}