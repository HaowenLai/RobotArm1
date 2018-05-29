#include "UsbCAN.hpp"
#include <stdio.h>

/* ***********************************************
 * CAN communication example
 *   must use `sudo` to run this program, and you 
 * should send 8 byte a frame, or the mini stm32 
 * may display abnormally. 
 * **********************************************/
int main()
{
    UsbCAN canII;
    VCI_CAN_OBJ can[100];

    if(canII.initCAN(UsbCAN::BAUDRATE_500K))
    {
        printf("init successfully\n");
    }
    
    /*
    //receive test
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

   
   
    //transmit test
    can[0].SendType = 0;
    can[0].DataLen = 8;
    can[0].Data[0] = 31;
    can[0].Data[1] = 33;
    can[0].Data[2] = 35;
    can[0].Data[3] = 37;
    can[0].Data[4] = 39;
    can[0].Data[5] = 41;
    can[0].Data[6] = 43;
    can[0].Data[7] = 45;
    can[0].ID = 0x0000;
    can[0].ExternFlag = 0;

    while(1)
    {
        int c;
        c=canII.transmit(can,1);
        printf("%d\n",c);
        getchar();
    }



   return 0;
}






