/* ********************************************************
 *   This file contains control algerithms that may be used 
 * in robotic arm control.
 * ********************************************************/

#include "control.hpp"
#include <stdio.h>
#include <unistd.h>

using namespace std;

void fixStepMove(std::vector<int>& oldVals, 
                  std::vector<int>& newVals,
                  UsbCAN& canDev,
                  int ID,
                  int step)
{
    if(oldVals.size()!=newVals.size())
    {
        printf("old and new value vectors do not have the same size\n");
        return;
    }

    while(1)
    {
        size_t satisfiedNum = 0;
        for(size_t i=0;i<oldVals.size();i++)
        {
            if(newVals[i]-oldVals[i] >= step)
                oldVals[i] += step;
            else if(newVals[i]-oldVals[i] <= -step)
                oldVals[i] += -step;
            else
                satisfiedNum++;
        }

        VCI_CAN_OBJ can;
        generateFrame(can,oldVals,ID);
        canDev.transmit(&can,1);

        if(satisfiedNum == oldVals.size())
            break;
        
        usleep(10*1000);
    }
}
