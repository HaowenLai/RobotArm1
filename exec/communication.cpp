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
#include <stdio.h>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace cv;

//multi-functional. Functions to choose
void swingPeriodically(UsbCAN& canII);
void singleMove(UsbCAN& canII);
void receiveFromCAN(UsbCAN& canII);
void matlabPredict(UsbCAN& canII);
void tfPredict(UsbCAN& canII);

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
           "5. tfPredict\n";
    
    int choose = 1;
    cin >> choose;
    switch(choose)
    {
      case 1:
        swingPeriodically(canII);
        break;
      case 2:
        while(1)
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
    //transmit test
    VCI_CAN_OBJ can;
    int step = 3;
    int pwmValue[4] {10,70,80,85};
    while(1)
    {
        if(pwmValue[0] > 130 || pwmValue[0] <10)
            step = -step;
        
        pwmValue[0]+=step;
        generateFrame(can,pwmValue,4);
        canII.transmit(&can,1);

        usleep(30*1000);
    }
}

//  It sends data to stm32 via CAN bus to make a single
//move for the robotic arms.
void singleMove(UsbCAN& canII)
{
    VCI_CAN_OBJ can;
    int pwmValue[4] {10,70,80,85};
    
    cin >> pwmValue[0] >> pwmValue[1];
    generateFrame(can,pwmValue,4);
    canII.transmit(&can,1);
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
    
    VCI_CAN_OBJ can;
    MatlabNetwork network(6);
    network.loadParams(dataPath,buffSize);
    Vec3d input;

    while(1)
    {
        cin >> input[0] >> input[1] >> input[2];
        Mat output = network.predict(input);
        vector<int> pwmValue{
            (int)output.at<double>(0),
            (int)output.at<double>(1)};
        cout<<"calculate:"<<output<<endl;
        
        generateFrame(can,pwmValue);
        canII.transmit(&can,1);
    }
}


//use tensorflow network to predict
void tfPredict(UsbCAN& canII)
{
    //! Network parameter
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "tf_network";
    string funcName   = "main";

    VCI_CAN_OBJ can;
    vector<double> inout{0.,0.};
    TfNetwork network(modulePath,moduleName,funcName);

    while(1)
    {
        cin >> inout[0] >> inout[1];
        network.callFunction(inout,inout);
        vector<int> pwmValue{
            (int)inout[0],
            (int)inout[1],
            80,85};
        cout<<"calculated motor value:"<<pwmValue[0]<<"  "<<pwmValue[1]<<endl;
        
        generateFrame(can,pwmValue);
        canII.transmit(&can,1);
    }
}
