/* ********************************************************
 *   This file includes class `Wifi` which uses socket to 
 * communicate. The class provides method that can send and
 * receive message.
 * @Author : Derek Lai
 * @Date   : 2018/7/5
 * @Version: v1.0
 * Copyright(c) All right reserved
 * ********************************************************/

#ifndef __WIFI_HPP__
#define __WIFI_HPP__

#include <string>

class Wifi
{
  public:
    enum WIFI_MSG
    {
        MSG_PREPARE_TO_DETECT,
        MSG_TARGET_QUALIFIED,
        MSG_TARGET_UNQUALIFIED,
        MSG_TARGET_IN_POSITION,
        MSG_FINISH_STAMPING,
        MSG_FINISH_ALL_JOB,
        MSG_ROGOR
    };

    Wifi(std::string servAddr,int port); //as client
    Wifi(int port,int clientNumber);     //as server
    ~Wifi();

    //client send message
    void sendMsg(WIFI_MSG msg);
    void sendMsg(unsigned char* msg,int length);
    //server send message
    void sendMsg(WIFI_MSG msg,int clientNumber);
    void sendMsg(unsigned char* msg,int length,int clientNumber);

    //client recv message
    bool recvNewMSG();
    bool recvNewMSG(unsigned char* msg,int length);
    //server recv message
    bool recvNewMSG(int clientNumber);
    bool recvNewMSG(unsigned char* msg,int length,int clientNumber);
  
  public:
    WIFI_MSG message;

  private:
    int wifi_type;      //specify the object,0 for client,1 for server
    int sock_fd;        //socket fd
    int clientNum;      //total numbers of client,for server
    int* client_fd;     //store client fd
};


#endif
