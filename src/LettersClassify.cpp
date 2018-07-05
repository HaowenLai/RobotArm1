/* ********************************************************
 *   This is a class for calling python to perform letters 
 * classification.
 * 
 * @Author : Derek Lai
 * @Date   : 2018/6/29
 * @Version: v1.0
 * Copyright(c) All right reserved
 * ********************************************************/

#include "LettersClassify.hpp"

using namespace std;
using namespace cv;

//-------------------------Class `LettersClassify` ------------------------
//public
LettersClassify::LettersClassify(string path,string moduleName,string funcName)
{
    Py_Initialize();
    if (!Py_IsInitialized()) 
        printf("initialize failed.\n");
    
    wchar_t* meaningless = NULL;
    wchar_t** meaningless1 = &meaningless;
    PySys_SetArgv(0, meaningless1);
    PyRun_SimpleString("import sys");
    
    path = "sys.path.append('" + path + "')";
    PyRun_SimpleString(path.c_str());
    
    //载入名为pyModuleName的脚本  
    pModule = PyImport_ImportModule(moduleName.c_str());
    if ( !pModule ||PyErr_Occurred()) 
    {
        printf("can't find py module");
        PyErr_Print();
    }

    //load all functions in the pModule
    pDict = PyModule_GetDict(pModule);  
    if ( !pDict )
    {
        printf("can't find the function dict");
    }

    // 找出函数名为main的函数
    pFunc = PyDict_GetItemString(pDict, funcName.c_str());
    if ( !pFunc || !PyCallable_Check(pFunc) )
    {
        printf("can't find function `main`");
    }
}

LettersClassify::~LettersClassify()
{
    Py_CLEAR(pModule);
    Py_CLEAR(pDict);
    Py_CLEAR(pFunc);
    Py_Finalize();
}


LettersClassify::Letters LettersClassify::detect(cv::Mat& InputImage)
{
    const int IMAGE_HEIGHT = 50;
    const int IMAGE_WIDTH  = 50;

    pArgs  = PyTuple_New(1);
    pImageData = PyList_New(0);
    
    Mat rzImage;
    resize(InputImage,rzImage,Size(IMAGE_HEIGHT,IMAGE_WIDTH));
    cvtColor(rzImage,rzImage,COLOR_BGR2RGB);
    
    for(int i=0; i<IMAGE_HEIGHT; i++)
    {
        for(int j=0; j<IMAGE_WIDTH; j++)
        {
            auto pixel = rzImage.at<Vec3b>(i,j);
            PyList_Append(pImageData,Py_BuildValue("i",pixel[0]));
            PyList_Append(pImageData,Py_BuildValue("i",pixel[1]));
            PyList_Append(pImageData,Py_BuildValue("i",pixel[2]));
        }
    }

    PyTuple_SetItem(pArgs, 0, pImageData);

    int rtVal;
    pRetVal = PyEval_CallObject(pFunc, pArgs);      //这里开始执行py脚本
    PyArg_ParseTuple(pRetVal, "i", &rtVal); //py脚本返回值给iRetVal

    Py_CLEAR(pArgs);
    Py_CLEAR(pImageData);
    Py_CLEAR(pRetVal);

    //deal with return
    switch(rtVal)
    {
      case 0:
        return LETTER_b;
      case 1:
        return LETTER_e;
      case 2:
        return LETTER_f;
      case 3:
        return LETTER_x;
      default:
        return NO_LETTER_ERROR;
    }
}


//--------------------end of Class `PyNetwork` -----------------------
