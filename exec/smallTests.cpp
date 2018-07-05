/* *************************************************************
 * CAN communication example
 *   Do not need to  use `sudo` to run this program if the rules
 * have been set in /etc/udev/rules.d/ .You should send 8 byte 
 * each frame, or the mini stm32 may display abnormally.
 *   This is a multi-functional program, you can choose one of 
 * its functions by entering the corresponding number while 
 * running. It won't stop until the program is killed.
 * ************************************************************/

#include "UsbCAN.hpp"
#include "BpNetwork.hpp"
#include "LettersClassify.hpp"
#include "ArucoMarker.hpp"
#include "RsVideoCapture.hpp"
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
void letterSort();

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
           "6. letterSort\n";
    
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
        letterSort();
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
    string funcName   = "bp_main";

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


//a test to LetterClassify.py
void letterSort()
{
    //! Network parameter
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "tf_network";
    string funcName   = "main";

    LettersClassify network(modulePath,moduleName,funcName);

    VideoCapture camera_rs(3);
    Mat img,img_detect;
    auto const area = Rect(100,100,50,50);
    namedWindow("window",WINDOW_AUTOSIZE);

    while((char)waitKey(50)!='q')
    {
        camera_rs >> img;
        img_detect = img(area);
        rectangle(img,area,Scalar(0,255,0),2);
        imshow("window",img);

        auto result = network.detect(img_detect);

        if(result == LettersClassify::LETTER_b)
            cout<<"yes~~~"<<endl;
        else
            cout<<"wrong!!@@##"<<endl;
    }
}