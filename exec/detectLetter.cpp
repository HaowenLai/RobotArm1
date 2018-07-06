/* *************************************************************
 *             Receive img and detect the letter
 *  This program runs as a client connected to server(i.e. mainTask)
 * It receives img and submit to a python network(i.e. LetterClassify)
 * After that, it will return the result to the server.
 * @Author : Derek Lai (LHW)
 * @Date   : 2018/7/5
 * Copyright(c) All right reserved
 * ************************************************************/

#include "Wifi.hpp"
#include "LettersClassify.hpp"

using namespace std;
using namespace cv;

int main()
{
    //! Network parameter
    char programName[100];
    getcwd(programName,100);    //get program path
    string modulePath(programName);
    modulePath = modulePath.substr(0,modulePath.find_last_of('/'))+"/src";
    string moduleName = "letter_classify";
    string funcName   = "main";
    LettersClassify network(modulePath,moduleName,funcName);

    Wifi c_wifi("127.0.0.1",1234);

    const int msgLength = 50*50*3;
    unsigned char msg_buff[msgLength];
    
    while(1)
    {
        while(!c_wifi.recvNewMSG());
        if(c_wifi.message == Wifi::MSG_FINISH_ALL_JOB)
            break;

        while(!c_wifi.recvNewMSG(msg_buff,msgLength));
        Mat img(50,50,CV_8UC3,msg_buff);
        
        switch(network.detect(img))
        {
        case LettersClassify::LETTER_b:
            cout<<"\n\nb\n\n";
            break;
        case LettersClassify::LETTER_e:
            cout<<"\n\ne\n\n";
            break;
        case LettersClassify::LETTER_f:
            cout<<"\n\nf\n\n";
            break;
        case LettersClassify::LETTER_x:
            cout<<"\n\nx\n\n";
            break;
        }
    }

    return 0;
}