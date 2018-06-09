/* *********************************************************
*   This is the definition for classs related to `BpNetwork`
*   It provides interface classes for networks calculated
* or built by Matlab and Python(tensorflow), namely 
* `MatlabNetwork` and `PyNetwork`
* *********************************************************/

#include "BpNetwork.hpp"
#include <fstream>
#include <cmath>

using namespace std;
using namespace cv;

//---------------------Class `MatlabNetwork` -------------------------
//public
MatlabNetwork::MatlabNetwork(int layerNumer):layerNum(layerNumer)
{ }

void MatlabNetwork::loadParams(std::string filename, int maxElementNum)
{
    ifstream fin(filename);
    double* buff = new double[maxElementNum];

    //load rows and cols for `netRowCols`
    for (int i = 0; i < layerNum; i++)
    {
        int temp1, temp2;
        fin >> temp1 >> temp2;
        netRowCols.push_back(Vint{ temp1, temp2 });
    }

    //load min and max for `InputMinMaxs`
    for (int i = 0; i < netRowCols[0][1]; i++)
    {
        double temp1, temp2;
        fin >> temp1 >> temp2;
        InputMinMaxs.push_back(Vdouble{ temp1, temp2 });
    }

    //load min and max for `OutputMinMaxs`
    for (int i = 0; i < netRowCols[layerNum-1][0]; i++)
    {
        double temp1, temp2;
        fin >> temp1 >> temp2;
        OutputMinMaxs.push_back(Vdouble{ temp1, temp2 });
    }

    //load layer weights for `layerWeights`
    for (const auto& rowCol : netRowCols)
    {
        for (int i = 0; i < rowCol[0] * rowCol[1]; i++)
            fin >> buff[i];

        layerWeights.push_back(Mat(rowCol[0], rowCol[1], CV_64FC1, buff).clone());  
    }

    //load bias for `layerBiases`
    for (auto& rowCol : netRowCols)
    {
        for (int i = 0; i < rowCol[0];i++)
            fin >> buff[i];
        layerBiases.push_back(Mat(rowCol[0], 1, CV_64FC1, buff).clone());
    }

    delete[] buff;
}


//private
cv::Mat MatlabNetwork::cvtInputType(const Vdouble& in)
{
    Mat input(in.size(), 1, CV_64FC1);

    int i = 0;
    for (auto x : in)
    {
        input.at<double>(i) = 
            2.0*(x - InputMinMaxs[i][0]) / (InputMinMaxs[i][1] - InputMinMaxs[i][0]) - 1.0;
        i++;
    }
    return input;
}

cv::Mat MatlabNetwork::cvtInputType(const Vec3d& in)
{
    Mat input(3, 1, CV_64FC1);

    for (int i = 0; i < 3;i++)
        input.at<double>(i) =
        2.0*(in[i] - InputMinMaxs[i][0]) / (InputMinMaxs[i][1] - InputMinMaxs[i][0]) - 1.0;
        
    return input;
}

void MatlabNetwork::tansig(Mat& x)
{
    auto it = x.begin<double>();
    auto itend = x.end<double>();

    while (it != itend)
    {
        *it = 2.0 / (1 + exp(-2.0 * *it)) - 1;
        it++;
    }
}

//--------------------end of Class `MatlabNetwork` -------------------



//-------------------------Class `PyNetwork` -------------------------
//public
TfNetwork::TfNetwork(string path,string moduleName,string funcName)
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

TfNetwork::~TfNetwork()
{
    Py_CLEAR(pModule);
    Py_CLEAR(pDict);
    Py_CLEAR(pFunc);
    Py_Finalize();
}


void TfNetwork::callFunction(std::vector<double>& input,
                             std::vector<double>& output)
{
    while(output.size()<2)
        output.push_back(0.);
    
    pArgs  = PyTuple_New(1);
    pListXdata = Py_BuildValue("[d,d]",input[0],input[1]);
    PyTuple_SetItem(pArgs, 0, pListXdata);

    pRetVal = PyEval_CallObject(pFunc, pArgs);      //这里开始执行py脚本
    PyArg_ParseTuple(pRetVal, "dd", &output[0],&output[1]); //py脚本返回值给iRetVal

    Py_CLEAR(pArgs);
    Py_CLEAR(pListXdata);
    Py_CLEAR(pRetVal);
}



//--------------------end of Class `PyNetwork` -----------------------
