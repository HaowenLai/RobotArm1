/* ********************************************************
 *   This file includes class `Wifi` which uses socket to 
 * communicate. The class provides method that can send and
 * receive message.
 * ********************************************************/

#include <iostream>
#include <string.h>

#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "Wifi.hpp"

using namespace std;

Wifi::Wifi(string servAddr,int port)
{
    wifi_type = 0;
    
    sock_fd = socket(PF_INET, SOCK_STREAM, 0);
    if(sock_fd == -1)
    {
        cout<< "cannot init socket!!\n";
        exit(0);
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr(servAddr.c_str());
    serv_addr.sin_port = htons(port);
    
    if(connect(sock_fd, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1)
    {
        cout << "connect error\n";
        exit(0);
    }
    else
        cout << "connected to server...\n\n";
}

Wifi::Wifi(int port,int clientNumber):clientNum(clientNumber)
{
    wifi_type = 1;
    client_fd = new int[clientNum];

    struct sockaddr_in s_in;//server address structure
    memset((void *)&s_in,0,sizeof(s_in));
    s_in.sin_family = AF_INET;//IPV4 communication domain
    s_in.sin_addr.s_addr = INADDR_ANY;//accept any address
    s_in.sin_port = htons(port);//change port to netchar
 
    sock_fd = socket(AF_INET,SOCK_STREAM,0);//socket(int domain, int type, int protocol)
    bind(sock_fd,(struct sockaddr *)&s_in,sizeof(s_in));
    listen(sock_fd,1);//only one connection in the same time

    cout<<"waiting for connection from client(s).."<<endl;
    for(int i = 0;i<clientNum;i++)
    {
        client_fd[i] = accept(sock_fd,nullptr,nullptr);
        cout<<"client "<< i <<" is connected"<<endl;
    }
    cout<<"all client(s) connected~~"<<endl;
}

Wifi::~Wifi()
{
    if(wifi_type == 0)
    {
        close(sock_fd);
    }
    else
    {
        for(int i=0;i<clientNum;i++)
            close(client_fd[i]);
        close(sock_fd);
        delete[] client_fd;
    }
}

//client send message
void Wifi::sendMsg(WIFI_MSG msg)
{
    if(wifi_type!=0)
    {
        cout<<"\n\n!! You use the wrong send function !\n";
        throw 0;
    }
    auto send_msg = (unsigned char)msg;
    write(sock_fd, &send_msg, 1);
}


//client send a long message
void Wifi::sendMsg(unsigned char* msg,int length)
{
    if(wifi_type!=0)
    {
        cout<<"\n\n!! You use the wrong send function !\n";
        throw 0;
    }
    write(sock_fd, msg, length);
    usleep(10*1000);
}

//server send message
void Wifi::sendMsg(WIFI_MSG msg,int clientNumber)
{
    if(wifi_type!=1)
    {
        cout<<"\n\n!! You use the wrong send function !\n";
        throw 1;
    }
    auto send_msg = (unsigned char)msg;
    write(client_fd[clientNumber],&send_msg,1);
}

//server send a long message
void Wifi::sendMsg(unsigned char* msg,int length,int clientNumber)
{
    if(wifi_type!=1)
    {
        cout<<"\n\n!! You use the wrong send function !\n";
        throw 1;
    }
    write(client_fd[clientNumber],msg,length);
    usleep(10*1000);
}

//client recv message
bool Wifi::recvNewMSG()
{
    if(wifi_type!=0)
    {
        cout<<"\n\n!! You use the wrong recv function !\n";
        throw 0;
    }

    unsigned char get_msg;
    auto len = read(sock_fd,&get_msg,1);

    if(len == 0)
        return false;
    else
    {
        message=(Wifi::WIFI_MSG)get_msg;
        return true;
    }
}

//client recv a long message
bool Wifi::recvNewMSG(unsigned char* msg,int length)
{
    if(wifi_type!=0)
    {
        cout<<"\n\n!! You use the wrong recv function !\n";
        throw 0;
    }

    int nCount=0;
    unsigned char buff[10000];
    while(nCount!=length)
    {
        auto len = read(sock_fd,buff,length);
        memcpy(msg+nCount,buff,len);
        nCount+=len;
    }

    if(nCount == 0)
        return false;
    else
        return true;
}

//server recv message
bool Wifi::recvNewMSG(int clientNumber)
{
    if(wifi_type!=1)
    {
        cout<<"\n\n!! You use the wrong recv function !\n";
        throw 1;
    }

    unsigned char get_msg;
    auto len = read(client_fd[clientNumber],&get_msg,1);

    if(len == 0)
        return false;
    else
    {
        message=(Wifi::WIFI_MSG)get_msg;
        return true;
    }
}

//server recv a long message
bool Wifi::recvNewMSG(unsigned char* msg,int length,int clientNumber)
{
    if(wifi_type!=1)
    {
        cout<<"\n\n!! You use the wrong recv function !\n";
        throw 1;
    }

    int nCount=0;
    unsigned char buff[10000];
    while(nCount!=length)
    {
        auto len = read(sock_fd,buff,length);
        memcpy(msg+nCount,buff,len);
        nCount+=len;
    }

    if(nCount == 0)
        return false;
    else
        return true;
}
