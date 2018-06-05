/* *************************************************************
 * CAN communication example
 *   must use `sudo` to run this program, and you should send
 * 8 byte each frame, or the mini stm32 may display abnormally. 
 * ************************************************************/

#include "UsbCAN.hpp"
#include "BpNetwork.hpp"
#include <stdio.h>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace cv;

//function for testing
void swingPeriodically(UsbCAN& canII);
void singleMove(UsbCAN& canII);



int main()
{
    UsbCAN canII;
    VCI_CAN_OBJ can;

    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"init successfully\n"
            <<"transmitting..."<<endl;
    }

    //single move test
    while(1)
    {
        singleMove(canII);
    }

    //reproject test
    // BpNetwork network(6);
    // network.loadParams("../data/network_data.txt",16*20);
    // Vec3d input;

    // while(1)
    // {
    //     cin >> input[0] >> input[1] >> input[2];
    //     Mat output = network.predict(input);
    //     vector<int> pwmValue{
    //         (int)output.at<double>(0),
    //         (int)output.at<double>(1)};
    //     cout<<"calculate:"<<output<<endl;
        
    //     generateFrame(can,pwmValue);
    //     canII.transmit(&can,1);
    // }

    return 0;
}


//This function will block the program until it ends
//It make the robotic arm1 swing forward and backward periodically
void swingPeriodically(UsbCAN& canII)
{
    //transmit test
    VCI_CAN_OBJ can;
    int step = 3;
    int pwmValue[2] {10,70};
    while(1)
    {
        if(pwmValue[0] > 130 || pwmValue[0] <10)
            step = -step;
        
        pwmValue[0]+=step;
        generateFrame(can,pwmValue,2);
        canII.transmit(&can,1);

        usleep(30*1000);
    }
}

//  It sends data to stm32 via CAN bus to make a single
//move for the robotic arms.
void singleMove(UsbCAN& canII)
{
    VCI_CAN_OBJ can;
    int pwmValue[2] {0};
    
    cin >> pwmValue[0] >> pwmValue[1];
    generateFrame(can,pwmValue,2);
    canII.transmit(&can,1);
}

/*
    //receive test
    //add it to the main function
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
*/
