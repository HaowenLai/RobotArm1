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
 * @Date   : 2018/7/18
 * ************************************************************/

#include "parameters.hpp"
#include "UsbCAN.hpp"
#include "BpNetwork.hpp"
#include "position.hpp"
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

ArucoMarker  rsMarker(vector<int> {8}, 
                      robot_arm::cameraParams::RS_CM,
                      robot_arm::cameraParams::RS_Dist);      //rs
CubePosition upperCube(robot_arm::cubePos::upperArea,
                       robot_arm::cubePos::upperValidLenMax,
                       robot_arm::cubePos::upperValidLenMin);//upper
CubePosition   armCube(robot_arm::cubePos::armArea,
                       robot_arm::cubePos::armValidLenMax,
                       robot_arm::cubePos::armValidLenMin);  //arm


RsVideoCapture camera_rs;

//global variables, to communicate with camera thread
auto ckeckSurfaceFlag = robot_arm::MISSION_OK;
Mat detectNumImg;
Mat detectBeforeResize;
    


//self calibration variables
double frontMmOffset = 0, upperMmOffset = 0;    //unit:mm
int frontPixelOffset = 0, upperPixelOffset = 0; //unit:pixel


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
    Wifi s_wifi(1234,1);

    //Arm1 initialize
    vector<int> motorValues;
    reset2initPos(motorValues,canII,1);
    cout << "Program is ready.\nPress <enter> to continue.." << endl;
    
    //------------------ begin algrithm -----------------------

    const int diff5_8 = 90; //height difference between #5 and #8
    
    //calibrate camera
    // selfCalibration(rsMarker,m2Marker2,
    //                 frontMmOffset,upperMmOffset,
    //                 frontPixelOffset,upperPixelOffset,canII,1);
    
again:
    cin.get();
    //get target position
    Vec3d targetFPos, targetUPos; //front position, upper position
    // double obstacleH;
    int motor1Val;
    cout<<"start to get target position...\n";
    while(true)
    {
        if(upperCube.cubeExist())
        {
            usleep(500*1000);
            auto targetPC = upperCube.getPosition();
            targetFPos = upperPC2frontAC(targetPC);
            targetUPos = upperPC2upperAC(targetPC);
            // obstacleH = obstacleHeight(camera_rs.DepthRaw, camera_rs.depth_scale,
            //     rsMarker.firstCorner(6),frontPixelOffset);
            motor1Val = motor1moveValue(targetUPos);
            
            cout<<"target position got ~\n";
            break;
        }
    }

    //deal with obstacle
    // if(obstacleH < 40)
    // {
    //     cout<<"avoid obstacle"<<endl;
    //     motorValues[1] = 230;
    //     fixStepMove(motorValues,canII,1);
        
    //     move2desiredPos(targetPos[0]-frontMmOffset,obstacleH-diff5_8-35,
    //                 motorValues,network,canII,1);
    // }
    // #ifdef _SINGLE_MOVE_MODE_
    // cin.get();
    // #endif

    //above the target, for more precise adjustment
    int x_offset = 7;
    move2desiredPos(targetFPos[0]-frontMmOffset+x_offset,targetFPos[1]-diff5_8-40,
                    motorValues,network,canII,1);
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    //motor #1 roughly move (rotate)
    motorValues[0] = motor1Val;
    fixStepMove(motorValues,canII,1);
    cout << motor1Val << endl;
    usleep(200*1000);
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif
    
    //adjust motor #6 (rotate)
    int marker4AdjustCount = 0;
    while(true)
    {
        const float epsilon = 0.05f;

        if(!armCube.isNewFrame())
            continue;

        if(armCube.cubeExist())
        {
            auto angle_offset = armCube.angle();
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
        }//end if(armCube.cubeExist())
        else if(marker4AdjustCount > 4)
        {
            //ensure upper label is in vision
            x_offset++;
            marker4AdjustCount = 0;
            move2desiredPos(targetFPos[0]-frontMmOffset+x_offset,targetFPos[1]-diff5_8-40,
                    motorValues,network,canII,1);
            
        }
        else
        {
            marker4AdjustCount++;
            // usleep(10*1000);
        }
    }
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

    
    //adjust motor #4(prepare for dig-into step)
    while(true)
    {
        const float epsilon = 10.0f;
        const float centerX = 410.0f;

        if(!armCube.isNewFrame())
            continue;

        if(armCube.cubeExist())
        {
            float targetX = armCube.getPosition().x;
            
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
        usleep(200*1000);
    }
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif


    //dig into the target
    move2desiredPos(targetFPos[0]-frontMmOffset+x_offset,targetFPos[1]-diff5_8-2,
                    motorValues,network,canII,1);
    #ifdef _SINGLE_MOVE_MODE_
    cin.get();
    #endif

/*
    //adjust motor #5 to vertical direction
    cout<<"start to adjust motor #5...\n";
    while(true)
    {
        const double epsilon = 16;

        if(!rsMarker.isNewFrame())
            continue;

        auto index8 = rsMarker.index(8);
        if(index8 != -1)
        {
            if(rsMarker.offset_tVecs[index8][0] - targetFPos[0] > epsilon)
            {
                motorValues[4] += 1;
                fixStepMove(motorValues,canII,1);
            }
            else if(rsMarker.offset_tVecs[index8][0] - targetFPos[0] < -epsilon)
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
    motorValues[6] = 190;
    fixStepMove(motorValues,canII,1);
    cout<<"grab the cube successfully ~\n";

    //check surface
    //check bottom surface
    getDetectImg(robot_arm::CHECK_BOTTOM_SURFACE,
                 ckeckSurfaceFlag,canII,1);
    s_wifi.sendMsg(Wifi::MSG_PREPARE_TO_DETECT,0);
    while(!s_wifi.recvNewMSG(0));
    s_wifi.sendMsg(detectNumImg.data,50*50*3,0);
    while(!s_wifi.recvNewMSG(0));
    if(s_wifi.message == Wifi::MSG_TARGET_QUALIFIED)
    {
        motorValues = vector<int> {97,80,95,125,235,165,190};
        evenVelMove(motorValues,canII,1);
        usleep(200*1000);

        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        usleep(200*1000);

        reset2initPos(motorValues,canII,1);
        goto checkCount;
    }
    else if(s_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
    {
        motorValues = vector<int> {153,65,95,125,235,165,190};
        evenVelMove(motorValues,canII,1);
        usleep(200*1000);

        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        usleep(200*1000);

        reset2initPos(motorValues,canII,1);
        goto checkCount;
    }
    
    //check back surface
    getDetectImg(robot_arm::CHECK_BACK_SURFACE,
                 ckeckSurfaceFlag,canII,1);
    s_wifi.sendMsg(Wifi::MSG_PREPARE_TO_DETECT,0);
    while(!s_wifi.recvNewMSG(0));
    s_wifi.sendMsg(detectNumImg.data,50*50*3,0);
    while(!s_wifi.recvNewMSG(0));
    if(s_wifi.message == Wifi::MSG_TARGET_QUALIFIED)
    {
        motorValues = vector<int> {97,80,95,125,235,165,190};
        evenVelMove(motorValues,canII,1);
        usleep(200*1000);

        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        usleep(200*1000);

        reset2initPos(motorValues,canII,1);
        goto checkCount;
    }
    else if(s_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
    {
        motorValues = vector<int> {153,65,95,125,235,165,190};
        evenVelMove(motorValues,canII,1);
        usleep(200*1000);

        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        usleep(200*1000);

        reset2initPos(motorValues,canII,1);
        goto checkCount;
    }
    
    //check front surface
    getDetectImg(robot_arm::CHECK_FRONT_SURFACE,
                 ckeckSurfaceFlag,canII,1);
    s_wifi.sendMsg(Wifi::MSG_PREPARE_TO_DETECT,0);
    while(!s_wifi.recvNewMSG(0));
    s_wifi.sendMsg(detectNumImg.data,50*50*3,0);
    while(!s_wifi.recvNewMSG(0));
    if(s_wifi.message == Wifi::MSG_TARGET_QUALIFIED)
    {
        motorValues = vector<int> {97,80,95,125,235,165,190};
        evenVelMove(motorValues,canII,1);
        usleep(200*1000);

        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        usleep(200*1000);
        
        reset2initPos(motorValues,canII,1);
        goto checkCount;
    }
    else if(s_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
    {
        motorValues = vector<int> {153,65,95,125,235,165,190};
        evenVelMove(motorValues,canII,1);
        usleep(200*1000);

        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        usleep(200*1000);

        reset2initPos(motorValues,canII,1);
        goto checkCount;
    }


    //move to passing position
    motorValues = vector<int> {130,53,133,96,219,39,190};
    evenVelMove(motorValues,canII,1);
    usleep(100*1000);
    //
    motorValues[2] = 136;
    fixStepMove(motorValues,canII,1);
    usleep(100*1000);
    //
    motorValues[6] = 124;
    fixStepMove(motorValues,canII,1);
    usleep(100*1000);
    //
    //loose clip
    motorValues[6] = 90;
    fixStepMove(motorValues,canII,1);
    reset2initPos(motorValues,canII,1);


    //check up surface
    getDetectImg(robot_arm::CHECK_UPPER_SURFACE,
                 ckeckSurfaceFlag,canII,1);
    s_wifi.sendMsg(Wifi::MSG_PREPARE_TO_DETECT,0);
    while(!s_wifi.recvNewMSG(0));
    s_wifi.sendMsg(detectNumImg.data,50*50*3,0);
    while(!s_wifi.recvNewMSG(0));
    if(s_wifi.message == Wifi::MSG_TARGET_QUALIFIED)
    {
        s_wifi.sendMsg(Wifi::MSG_TARGET_QUALIFIED,0);
    }
    else if(s_wifi.message == Wifi::MSG_TARGET_UNQUALIFIED)
    {
        s_wifi.sendMsg(Wifi::MSG_TARGET_UNQUALIFIED,0);
    }
    else
    {
        s_wifi.sendMsg(Wifi::MSG_TARGET_IN_POSITION,0);
    }
    
    
    //black arm to grab the cube
    while(!s_wifi.recvNewMSG(0));
    if(s_wifi.message == Wifi::MSG_FINISH_A_JOB)
    {
        s_wifi.sendMsg(Wifi::MSG_START_A_NEW_JOB,0);
        goto checkCount;
    }

    

/*    
    //move to unqualified place
    motorValues[0] = 154;
    motorValues[3] = 125;
    motorValues[4] = 255;
    motorValues[5] = 165;
    evenVelMove(motorValues,canII,1);
    //
    motorValues[1] = 62;
    motorValues[2] = 89;
    evenVelMove(motorValues,canII,1);

    //loose clip
    motorValues[6] = 90;
    fixStepMove(motorValues,canII,1);
    
    //reset
    reset2initPos(motorValues,canII,1);
*/

/*
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
        move2desiredPos(targetPos[0]-frontMmOffset,-20.0-diff5_8,
                        motorValues,network,canII,1);
        motorValues[0] = 127;
        motorValues[3] = 125;
        motorValues[4] = 255;
        motorValues[5] = 165;
        evenVelMove(motorValues,canII,1);
        move2desiredPos(175-frontMmOffset,-18.0-diff5_8,
                        motorValues,network,canII,1);
        cout<<"finish moving the cube ~\n";
        
        //loose the clip
        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        
        //wait for arm0 to finish stamping
        move2desiredPos(60-frontMmOffset,-160,motorValues,network,canII,1);
        s_wifi.sendMsg(Wifi::MSG_TARGET_IN_POSITION,1);
        while(!s_wifi.recvNewMSG(1));

        //move the stamped cube to "correct" place
        //get target position
        cout<<"start to get stamped target position\n";
        while(true)
        {
            if(rsMarker.index(6) != -1 && m2Marker2.index(4)!=-1)
            {
                targetPos = rsMarker.offset_tVecs[rsMarker.index(6)];
                motor1Val = motor1moveValue(m2Marker2.offset_tVecs[m2Marker2.index(4)],
                    upperPixelOffset);
                
                cout<<"target position got ~\n";
                break;
            }
        }
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif

        //above the target, for more precise adjustment
        x_offset = 0;
        move2desiredPos(targetPos[0]-frontMmOffset+x_offset,targetPos[1]-diff5_8-35,
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
                move2desiredPos(targetPos[0]-frontMmOffset+x_offset,targetPos[1]-diff5_8-35,
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
            const float epsilon = 0.4f;
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
            usleep(200*1000);
        }
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif


        //dig into the target
        move2desiredPos(targetPos[0]-frontMmOffset+x_offset,targetPos[1]-diff5_8-7,
                        motorValues,network,canII,1);
        usleep(500*1000);
        #ifdef _SINGLE_MOVE_MODE_
        cin.get();
        #endif

        //use #7 to grab the cube
        cout<<"start to grab the cube\n";
        motorValues[6] = 140;
        fixStepMove(motorValues,canII,1);
        cout<<"grab the cube successfully ~\n";

        //move to "correct" place
        move2desiredPos(targetPos[0]-frontMmOffset+x_offset,targetPos[1]-diff5_8-45,
                        motorValues,network,canII,1);
        motorValues = vector<int> {94,68,92,125,255,165,140};
        evenVelMove(motorValues,canII,1);

        //loose clip
        motorValues[6] = 90;
        fixStepMove(motorValues,canII,1);
        
        //reset
        reset2initPos(motorValues,canII,1);
    }
*/
    //whether it has finished all jobs
checkCount:
    if(runCount < runTimes)
    {
        runCount++;
        goto again;
    }

    //wait...
    cout<<"\n~~~ I FINISH MY JOB ~~~\n-- press 'q' to exit --\n";
    // s_wifi.sendMsg(Wifi::MSG_FINISH_ALL_JOB,0);
    // s_wifi.sendMsg(Wifi::MSG_FINISH_ALL_JOB,1);
    pthread_join(cameraThread, NULL);
    
    return 0;
}



//camera thread for func `moveInRoute`
static void* camera_thread(void* data)
{
    //number detection area
    const Size detectSize(50,50);
    const Rect bottomArea(Point(382,67),Point(410,115));
    const Rect backArea(Point(341,177),Point(367,220));
    const Rect frontArea(Point(325,155),Point(365,195));
    const Rect upperArea(Point(235,205),Point(281,260));
    
    
    Mat rsImg, armImg, upperImg;
    VideoCapture   armCamera(4);    //arm camera
    VideoCapture upperCamera(3);    //upper camera

    #ifndef _NO_VIDEO_MODE_
    namedWindow("view_front", WINDOW_AUTOSIZE);
    namedWindow("view_arm", WINDOW_AUTOSIZE);
    namedWindow("view_up", WINDOW_AUTOSIZE);
    #endif

    while((char)waitKey(30) != 'q')
    {
        camera_rs   >> rsImg;
        armCamera   >> armImg;
        upperCamera >> upperImg;
        
        rsMarker.detect(rsImg);
        armCube.detect(armImg);
        upperCube.detect(upperImg);

        #ifndef _NO_VIDEO_MODE_
        rsMarker.outputOffset(rsImg,Point(30,30));
        armCube.drawBoundry(armImg);
        armCube.outputPos(armImg,Point(30,30));
        upperCube.drawBoundry(upperImg);
        upperCube.outputPos(upperImg,Point(30,30));
        imshow("view_front",rsImg);
        imshow("view_arm",armImg);
        imshow("view_up",upperImg);
        #endif

        //take detect roi
        if(ckeckSurfaceFlag == robot_arm::CHECK_BOTTOM_SURFACE)
        {
            detectBeforeResize = rsImg(bottomArea).clone();
            resize(detectBeforeResize,detectNumImg,detectSize);
            cout<<detectNumImg.cols<<" "<<detectNumImg.rows<<endl;
            imshow("number",detectNumImg);
            ckeckSurfaceFlag = robot_arm::MISSION_OK;
        }
        else if(ckeckSurfaceFlag == robot_arm::CHECK_BACK_SURFACE)
        {
            detectBeforeResize = rsImg(backArea).clone();
            resize(detectBeforeResize,detectNumImg,detectSize);
            imshow("number",detectNumImg);
            ckeckSurfaceFlag = robot_arm::MISSION_OK;
        }
        else if(ckeckSurfaceFlag == robot_arm::CHECK_FRONT_SURFACE)
        {
            detectBeforeResize = rsImg(frontArea).clone();
            resize(detectBeforeResize,detectNumImg,detectSize);
            imshow("number",detectNumImg);
            ckeckSurfaceFlag = robot_arm::MISSION_OK;
        }
        else if(ckeckSurfaceFlag == robot_arm::CHECK_UPPER_SURFACE)
        {
            detectBeforeResize = upperImg(upperArea).clone();
            resize(detectBeforeResize,detectNumImg,detectSize);
            imshow("number",detectNumImg);
            ckeckSurfaceFlag = robot_arm::MISSION_OK;
        }
    }

    pthread_exit(0);
}
