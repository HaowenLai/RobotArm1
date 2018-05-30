/* ***********************************************************
 * Head file for UsbCAN.cpp
 * @description: class to operate ZLG USBCAN device.  
 * @author : Derek Lai
 * @date   : 2018/5/29
 * @version: 1.2
 * Copyright(c) All right reserved
** ************************************************************/

#ifndef __USBCAN_HPP__
#define __USBCAN_HPP__

typedef struct _VCI_CAN_OBJ{
    unsigned int  ID;
    unsigned int TimeStamp;
    unsigned char TimeFlag;
    unsigned char SendType;
    unsigned char RemoteFlag;//是否是远程帧
    unsigned char ExternFlag;//是否是扩展帧
    unsigned char DataLen;
    unsigned char Data[8];
    unsigned char Reserved[3];
}VCI_CAN_OBJ,*PVCI_CAN_OBJ;


class UsbCAN
{
  public:
    enum DeviceType
    {
        VCI_PCI5121 = 1,
        VCI_PCI9810 = 2,
        VCI_USBCAN1 = 3,
        VCI_USBCAN2 = 4,
        VCI_PCI9820 = 5,
        VCI_CAN232  = 6,
        VCI_PCI5110 = 7,
        VCI_CANLite = 8,
        VCI_ISA9620 = 9,
        VCI_ISA5420 = 10
    };

    enum Baudrate
    {
        BAUDRATE_5K   = 0xFFBF,
        BAUDRATE_10K  = 0x1C31,
        BAUDRATE_20K  = 0x1C18,
        BAUDRATE_50K  = 0x1C09,
        BAUDRATE_125K = 0x1C03,
        BAUDRATE_250K = 0x1C01,
        BAUDRATE_500K = 0x1C00,
        BAUDRATE_800K = 0x1600,
        BAUDRATE_1M   = 0x1400
    };


    UsbCAN(DeviceType dt = VCI_USBCAN2,
           unsigned int devId = 0);
    ~UsbCAN();
    bool initCAN(Baudrate baudrate,unsigned int channel=0);
    //return number of frames received
    unsigned int receive(PVCI_CAN_OBJ pCan,unsigned int len,unsigned int channel=0);
    //return frames send successfully
    unsigned int transmit(PVCI_CAN_OBJ pCan,unsigned int len,unsigned int channel=0); 

  private:
    DeviceType devType;
    unsigned int devId;
    
    bool openDevice();
    bool closeDevice();
    
};


void generateFrame(VCI_CAN_OBJ& canObj,int* data,int dataLen,
    int id=0,int sendType=0,int externFlag=0,int remoteFlag=0);

#endif
