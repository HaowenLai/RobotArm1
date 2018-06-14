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
#include "control.hpp"
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
    vector<int> pwmValue0 {128,128,135,140};
    vector<int> newValue0 {128,128,135,140};
    //Arm1
    vector<int> pwmValue1 {127,255,50,125,235,170,128};
    vector<int> newValue1 {127,255,50,125,235,170,128};

    //initialize
    fixStepMove(pwmValue0,newValue0,canII,0);
    fixStepMove(pwmValue1,newValue1,canII,1);
    
    while(1)
    {
        cin >> newValue1[1] >> newValue1[2];
        fixStepMove(pwmValue1,newValue1,canII,1);
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
        
        fixStepMove(pwmValue,newValue,canII,0);
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
    vector<int> pwmValue1 {127,255,50,125,235,170,100};
    vector<int> newValue1 {127,255,50,125,235,170,100};
    fixStepMove(pwmValue1,newValue1,canII,1);

    while(1)
    {
        cin >> inout[0] >> inout[1];
        network.callFunction(inout,inout);
        newValue1[1] = (int)inout[0];
        newValue1[2] = (int)inout[1];

        cout<<"calculated motor value:"<<newValue1[1]<<"  "<<newValue1[2]<<endl;
        
        fixStepMove(pwmValue1,newValue1,canII,1);
    }
}

//move along a certain route
void moveInRoute(UsbCAN& canII)
{
    //! Network parameter
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "tf_network";
    string funcName   = "main";

    vector<double> inout{0.,0.};
    TfNetwork network(modulePath,moduleName,funcName);

    //Arm1 initialize
    vector<int> pwmValue1 {127,255,50,125,235,170,100};
    vector<int> newValue1 {127,255,50,125,235,170,100};
    fixStepMove(pwmValue1,newValue1,canII,1);

    //camera instrinc matrix and distort coefficients
    const Mat M2_cameraMatrix = (Mat_<double>(3, 3) 
        << 1208.33,0, 303.71, 0, 1209.325, 246.98, 0, 0, 1);
    const Mat M2_distCoeffs = (Mat_<double>(1, 5)
        << -0.3711,-4.0299, 0, 0,22.9040);

    Mat img;
    ArucoMarker m2Marker(vector<int>({5,6,8}), M2_cameraMatrix, M2_distCoeffs);

    VideoCapture camera(0);
    namedWindow("view", WINDOW_AUTOSIZE);
    
    //get target position
    Vec3d targetPos;
    cout<<"start to get target position...\n";
    while(1)
    {
        camera >> img;
        m2Marker.detect(img);
        m2Marker.outputOffset(img,Point(30,30));
        imshow("view",img);
        waitKey(20);

        if(m2Marker.offset_tVecs.size()==3)
        {
            targetPos = m2Marker.offset_tVecs[1];
            cout<<"target position got ~\n";
            break;
        }
    }

    //move arm1 to target position
    inout[0] = targetPos[0];
    inout[1] = targetPos[1]-95; //#8 is 86 units above target

    network.callFunction(inout,inout);
    newValue1[1] = (int)inout[0];
    newValue1[2] = (int)inout[1];
    
    cout<<"calculated motor value:"<<newValue1[1]<<"  "<<newValue1[2]<<endl;
    cout<<"start to move arm roughly...\n";
    fixStepMove(pwmValue1,newValue1,canII,1);
    cout<<"finish moving arm roughly...\n";

    //adjust #5 to vertical direction
    cout<<"start to adjust #5...\n";
    const double epsilon = 3;
    while(1)
    {
        camera >> img;
        m2Marker.detect(img);
        m2Marker.outputOffset(img,Point(30,30));
        imshow("view",img);
        waitKey(20);

        if(m2Marker.offset_tVecs.size()==2)
        {
            if(m2Marker.offset_tVecs[1][0] - targetPos[0] > epsilon)
            {
                newValue1[4] += 1;
                fixStepMove(pwmValue1,newValue1,canII,1);
            }
            else if(m2Marker.offset_tVecs[1][0] - targetPos[0] < -epsilon)
            {
                newValue1[4] -= 1;
                fixStepMove(pwmValue1,newValue1,canII,1);
            }
            else
            {
                cout<<"#5 is in position ~\n";
                break;
            }
        }//end if(size==3)
    }

    //use #7 to grab the cube
    cout<<"start to grab the cube\n";
    newValue1[6] = 130;
    fixStepMove(pwmValue1,newValue1,canII,1);
    cout<<"grab the cube successfully ~\n";

    //move the cube to another place
    newValue1[1] = 250;
    newValue1[2] = 50;
    fixStepMove(pwmValue1,newValue1,canII,1);

    //wait...
    while(1)
    {
        camera >> img;
        m2Marker.detect(img);
        m2Marker.outputOffset(img,Point(30,30));
        imshow("view",img);
        if((char)waitKey(20) == 'q')
            break;
    }

}
