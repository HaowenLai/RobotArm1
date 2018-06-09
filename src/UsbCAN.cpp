/* *********************************************************
 *   this file use provides functions to operate ZLG USBCAN.
 * ********************************************************/

//extern "C"{
//#include "__usbcan.h"
//}
#include "UsbCAN.hpp"
#include <stdio.h>

//----------------------h file provided by ZLG -----------------
extern "C"{

typedef unsigned short int USHORT;
typedef unsigned char      BYTE;
typedef char               CHAR;
typedef unsigned char      UCHAR;
typedef unsigned int       UINT;
typedef unsigned int       DWORD;
typedef void*              PVOID;
typedef unsigned int       ULONG;
typedef int                INT;

//定义初始化CAN的数据类型
typedef struct _INIT_CONFIG{
	DWORD	AccCode;
	DWORD	AccMask;
	DWORD	Reserved;
	UCHAR	Filter;
	UCHAR	Timing0;	
	UCHAR	Timing1;	
	UCHAR	Mode;
}VCI_INIT_CONFIG,*PVCI_INIT_CONFIG;

DWORD VCI_OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
DWORD VCI_CloseDevice(DWORD DeviceType,DWORD DeviceInd);
DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);
DWORD VCI_StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

ULONG VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);
ULONG VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);

}
//END-------------------h file provided by ZLG -----------------




//-------------------- Definition of class `UsbCAN`------------------
//public
UsbCAN::UsbCAN(DeviceType dt,unsigned int devId):
	devType(dt),
	devId(devId)
{
	if(openDevice())
		printf("open device successfully\n");

}

UsbCAN::~UsbCAN()
{
	closeDevice();
}

bool UsbCAN::initCAN(Baudrate baudrate,unsigned int channel)
{
    VCI_INIT_CONFIG config;
    //the following four options are not avaliable in USBCAN-II
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter  = 1;
    config.Mode    = 0;

    config.Timing0 = (UCHAR)baudrate & 0xff;
    config.Timing1 = (UCHAR)baudrate >> 8;

    VCI_InitCAN((DWORD)devType,devId, channel, &config);
    if(VCI_StartCAN((DWORD)devType, devId, channel))
        return true;
    else
        return false;
}

unsigned int UsbCAN::receive(PVCI_CAN_OBJ pCan,unsigned int len,unsigned int channel)
{
	return VCI_Receive((DWORD)devType,devId,channel,pCan,len,20);
}

unsigned int UsbCAN::transmit(PVCI_CAN_OBJ pCan,unsigned int len,unsigned int channel)
{
	return VCI_Transmit((DWORD)devType,devId,channel,pCan,len);
}


//private
bool UsbCAN::openDevice()
{
    if(VCI_OpenDevice((DWORD)devType,devId,0))
        return true;
    else
        return false;
}


bool UsbCAN::closeDevice()
{
    if(VCI_CloseDevice((DWORD)devType,devId))
        return true;
    else
        return false;
}
//END----------------- Definition of class `UsbCAN`------------------


void generateFrame(VCI_CAN_OBJ& canObj,int* data,int dataLen,
    int id,int sendType,int externFlag,int remoteFlag)
{
    canObj.ID        = (unsigned int)id;
    canObj.SendType  = (unsigned int)sendType;
    canObj.ExternFlag= (unsigned char)externFlag;
    canObj.RemoteFlag= (unsigned char)remoteFlag;

    canObj.DataLen   = (unsigned char)dataLen;
    for(int i=0;i<dataLen;i++)
    {
        canObj.Data[i] = data[i] & 0xff;
    }
}

void generateFrame(VCI_CAN_OBJ& canObj,std::vector<int>& data,
    int id,int sendType,int externFlag,int remoteFlag)
{
    canObj.ID        = (unsigned int)id;
    canObj.SendType  = (unsigned int)sendType;
    canObj.ExternFlag= (unsigned char)externFlag;
    canObj.RemoteFlag= (unsigned char)remoteFlag;
    canObj.DataLen   = (unsigned char)data.size();

    int i=0;
    for(int x:data)
    {
        canObj.Data[i] = x & 0xff;
        i++;
    }
}