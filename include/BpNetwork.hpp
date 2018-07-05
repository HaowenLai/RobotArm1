/* ********************************************************
 * These are classes for bp network.
 *   It provides interface classes for networks calculated
 * or built by Matlab and Python(tensorflow), namely 
 * `MatlabNetwork` and `TfNetwork`
 * @Author : Derek Lai
 * @Date   : 2018/7/4
 * @Version: v3.0
 * Copyright(c) All right reserved
 * ********************************************************/

#ifndef __BPNETWORK_HPP__
#define __BPNETWORK_HPP__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Python.h>

class MatlabNetwork
{
  private:
    typedef std::vector<int>                Vint;
    typedef std::vector<Vint>               VVint;
    typedef std::vector<double>             Vdouble;
    typedef std::vector<Vdouble>            VVdouble;
    typedef std::vector<cv::Mat>            VMat;
    

  public:
    MatlabNetwork(int layerNumer);

    //@The format for data file should be:
    //rows1(3)  cols1(3)
    //INmin1    INmax1...
    //OUTmin1   OUTmax1...
    //w1 w2 w3
    //b1 b2 b3
    void loadParams(std::string filename, int maxElementNum);
  
    template<typename inputType>
    //inputType can only be Vdouble or Vec3d
    cv::Mat predict(const inputType& input);


  private:  
    cv::Mat cvtInputType(const Vdouble& in);
    cv::Mat cvtInputType(const cv::Vec3d& in);
    void tansig(cv::Mat& x);

    const int layerNum;
    VVint netRowCols;
    VVdouble InputMinMaxs;
    VVdouble OutputMinMaxs;
    VMat layerWeights;
    VMat layerBiases;
};

//definition of template function `BpNetwork::predict`
template<typename inputType>
cv::Mat MatlabNetwork::predict(const inputType& input)
{
    cv::Mat output = cvtInputType(input);

    for (int i = 0; i < layerNum; i++)
    {
        output = layerWeights[i] * output + layerBiases[i];
        if (i<layerNum - 1)
            tansig(output);
    }

    //convert output range
    for (int i = 0; i < netRowCols[layerNum - 1][0]; i++)
    {
        output.at<double>(i) =
            (output.at<double>(i) +1)*(OutputMinMaxs[i][1] - OutputMinMaxs[i][0])
            / 2.0 + OutputMinMaxs[i][0];
    }
    
    return output;
}



class TfNetwork
{
 public:
    TfNetwork(std::string path,
              std::string moduleName,
              std::string funcName);
    ~TfNetwork();
    
    //position predict
    //input and output can be the same variable
    void callFunction(std::vector<double>& input,
                      std::vector<double>& output);

  private:
    PyObject* pModule;      //module obj
    PyObject* pDict;        //fuction dict obj
    PyObject* pFunc;        //position function
    PyObject* pArgs;        //target function argument
    PyObject* pRetVal;      //return value
    
    PyObject* pListXdata;   //X_data in List type
};


#endif
