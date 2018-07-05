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
    string modulePath = "/home/savage/workspace/cpp_ws/Aruco-marker/src";
    string moduleName = "letter_classify";
    string funcName   = "main";
    LettersClassify network(modulePath,moduleName,funcName);

    Wifi c_wifi("127.0.0.1",1234);

    const int msgLength = 50*50*3;
    auto msg_buff = new unsigned char[msgLength];
    while(!c_wifi.recvNewMSG(msg_buff,msgLength));

    Mat img(50,50,CV_8UC3,msg_buff);
    
    switch(network.detect(img))
    {
      case LettersClassify::LETTER_b:
        cout<<"\n\nb\n";
        break;
      case LettersClassify::LETTER_e:
        cout<<"\n\ne\n";
        break;
      case LettersClassify::LETTER_f:
        cout<<"\n\nf\n";
        break;
      case LettersClassify::LETTER_x:
        cout<<"\n\nx\n";
        break;
    }

    return 0;
}