/* ******************************************************
 * This is a class for bp network.
 * It can read the weights and bias calculated by Matlab.
 * @Author : Derek Lai
 * @Date   : 2018/6/5
 * @Version: v1.3
 * ******************************************************/

#ifndef __BPNETWORK_HPP__
#define __BPNETWORK_HPP__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class BpNetwork
{
private:
    typedef std::vector<int>                Vint;
    typedef std::vector<Vint>               VVint;
    typedef std::vector<double>             Vdouble;
    typedef std::vector<Vdouble>            VVdouble;
    typedef std::vector<cv::Mat>            VMat;
    

public:
    BpNetwork(int layerNumer);

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
cv::Mat BpNetwork::predict(const inputType& input)
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


#endif