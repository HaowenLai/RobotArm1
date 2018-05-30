#include "UsbCAN.hpp"
#include <stdio.h>
#include <iostream>
#include <unistd.h>

using namespace std;
/* ***********************************************
 * CAN communication example
 *   must use `sudo` to run this program, and you 
 * should send 8 byte a frame, or the mini stm32 
 * may display abnormally. 
 * **********************************************/
int main()
{
    UsbCAN canII;
    VCI_CAN_OBJ can[10];

    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        cout<<"init successfully\n"
            <<"waiting..."<<endl;
    }
    

    //transmit test
    int step = 3;
    int pwmValue = 10;
    while(1)
    {
        if(pwmValue > 130 || pwmValue <10)
            step = -step;
        
        pwmValue+=step;
        generateFrame(*can,&pwmValue,1);
        canII.transmit(can,1);

        usleep(30*1000);
        //getchar();
    }



   return 0;
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
