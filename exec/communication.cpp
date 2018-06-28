/* *************************************************************
 * CAN communication example
 *   Must use `sudo` to run this program, and you should send
 * 8 byte each frame, or the mini stm32 may display abnormally.
 *   This is a multi-functional program, you can choose one of 
 * its functions by entering the corresponding number while 
 * running. It won't stop until the program is killed.
 * ************************************************************/

#include "UsbCAN.hpp"
#include "BpNetwork.hpp"
#include "ArucoMarker.hpp"
#include "RsVideoCapture.hpp"
#include "control.hpp"

#include <pthread.h>
#include <stdio.h>
#include <iostream>


using namespace std;
using namespace cv;

//multi-functional. Functions to choose
void swingPeriodically(UsbCAN& canII);
void singleMove(UsbCAN& canII);
void receiveFromCAN(UsbCAN& canII);
void matlabPredict(UsbCAN& canII);
void tfPredict(UsbCAN& canII);
void moveInRoute(UsbCAN& canII);

int main()
{
    UsbCAN canII;
    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"init successfully\n"
            <<"transmitting..."<<endl;
    }

    cout<< "Enter the number to choose a function:\n"
           "1. swingPeriodically\n"
           "2. singleMove\n"
           "3. receiveFromCAN\n"
           "4. matlabPredict\n"
           "5. tfPredict\n"
           "6. moveInRoute\n";
    
    int choose = 1;
    cin >> choose;
    switch(choose)
    {
      case 1:
        swingPeriodically(canII);
        break;
      case 2:
        singleMove(canII);
        break;
      case 3:
        receiveFromCAN(canII);
        break;
      case 4:
        matlabPredict(canII);
        break;
      case 5:
        tfPredict(canII);
        break;
      case 6:
        moveInRoute(canII);
        break;
      default:
        cout<<"your choice is not in the range! Exit.\n";
        break;
    }
    
    return 0;
}


//This function will block the program until it ends
//It make the robotic arm1 swing forward and backward periodically
void swingPeriodically(UsbCAN& canII)
{
    VCI_CAN_OBJ can;
    int step = 1;
    int pwmValue[4] {128,128,135,140};

    //initialize arm1
    int arm1Value[] {127,255,50,125,235,170,128};
    generateFrame(can,arm1Value,7,1);
    canII.transmit(&can,1);

    while(1)
    {
        if(pwmValue[0] > 130 || pwmValue[0] <10)
            step = -step;
        
        pwmValue[0]+=step;
        generateFrame(can,pwmValue,4);
        canII.transmit(&can,1);

        usleep(10*1000);
    }
}

//  It sends data to stm32 via CAN bus to make a single
//move for the robotic arms.
void singleMove(UsbCAN& canII)
{
    //Arm0
    // vector<int> pwmValue0 {128,128,135,140};
    // vector<int> newValue0 {128,128,135,140};
    //Arm1
    // vector<int> pwmValue1 {127,255,50,125,235,170,128};
    vector<int> newValue1;

    //initialize
    reset2initPos(newValue1,canII,1);
    // fixStepMove(pwmValue0,newValue0,canII,0);
    // fixStepMove(pwmValue1,newValue1,canII,1);
    
    while(1)
    {
        cin >> newValue1[1] >> newValue1[2];
        fixStepMove(newValue1,canII,1);
    }
}

//block and receive msg from CAN
void receiveFromCAN(UsbCAN& canII)
{
    VCI_CAN_OBJ can[50];

    while(1)
    {
        int count = canII.receive(can,50);
        if(!count)
        {
            continue;
        }

        printf("count is %d\n",count);
        for(int i=0;i<count;i++)
        {
            unsigned char len = can[i].DataLen;
            printf("ID is %011x\n",can[i].ID);
            printf("data len %d\n",len);
            printf("timestamp %d\n",can[i].TimeStamp);
            for(int j = 0;j<len;j++)
            {
                printf("%02d ",can[i].Data[j]);
            }
            printf("\n\n");
        }
    }
}


//use matlab network to predict
void matlabPredict(UsbCAN& canII)
{
    //! Network parameter
    string dataPath = "../data/network_data.txt";
    int buffSize = 16*20;
    
    MatlabNetwork network(6);
    network.loadParams(dataPath,buffSize);
    Vec3d input;

    vector<int> pwmValue {128,128,135,140};

    while(1)
    {
        cin >> input[0] >> input[1] >> input[2];
        Mat output = network.predict(input);
        vector<int> newValue{
            (int)output.at<double>(0),
            (int)output.at<double>(1),
            135,140};
        cout<<"calculate:"<<output<<endl;
        
        fixStepMove(newValue,canII,0);
    }
}


//use tensorflow network to predict
void tfPredict(UsbCAN& canII)
{
    //! Network parameter
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "tf_network";
    string funcName   = "main";

    vector<double> inout{0.,0.};
    TfNetwork network(modulePath,moduleName,funcName);

    //Arm1
    vector<int> newValue1;
    reset2initPos(newValue1,canII,1);

    while(1)
    {
        cin >> inout[0] >> inout[1];
        network.callFunction(inout,inout);
        newValue1[1] = (int)inout[0];
        newValue1[2] = (int)inout[1];

        cout<<"calculated motor value:"<<newValue1[1]<<"  "<<newValue1[2]<<endl;
        
        fixStepMove(newValue1,canII,1);
    }
}


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

//camera thread for func `moveInRoute`
static void* camera_thread(void* data)
{
    Mat img0, img1, img2;
    VideoCapture camera1(4);
    VideoCapture camera2(3);
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
    }

    pthread_exit(0);
}

// #define _DEBUG_MODE_

//move along a certain route
void moveInRoute(UsbCAN& canII)
{

    //create camera thread
    pthread_t cameraThread;
    pthread_create(&cameraThread, NULL, camera_thread, NULL);

    //Arm1 initialize
    vector<int> newValue1;
    reset2initPos(newValue1,canII,1);
    cin.get();cin.get();

    //! Network parameter
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "tf_network";
    string funcName   = "main";

    vector<double> inout{0.,0.};
    TfNetwork network(modulePath,moduleName,funcName);

    //-------------- begin algrithm -----------------------

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
        move2desiredPos(targetPos[0],obstacleH-diff5_8-30,
                    newValue1,network,canII,1);
    }
#ifdef _DEBUG_MODE_
    cin.get();
#endif

    //above the target, for more precise adjustment
    int x_offset = 10;
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
        const double epsilon = 2;

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
    }
#ifdef _DEBUG_MODE_
    cin.get();
#endif

    //use #7 to grab the cube
    cout<<"start to grab the cube\n";
    newValue1[6] = 130;
    fixStepMove(newValue1,canII,1);
    cout<<"grab the cube successfully ~\n";

    //move the cube to another place (92,9)
    move2desiredPos(targetPos[0],-1.0-diff5_8,
                    newValue1,network,canII,1);
    newValue1[0] = 127;
    newValue1[5] = 165;
    newValue1[3] = 125;
    move2desiredPos(92.0,9.0-diff5_8,
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

}
