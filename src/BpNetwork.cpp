/* ******************************************************
* This is the definition for class `BpNetwork`
* It can read the weights and bias calculated by Matlab.
* ******************************************************/

#include "BpNetwork.hpp"
#include <fstream>
#include <cmath>

using namespace std;
using namespace cv;

//-----------------------Class `BpNetwork` -------------------------
//public
BpNetwork::BpNetwork(int layerNumer):layerNum(layerNumer)
{ }

void BpNetwork::loadParams(std::string filename, int maxElementNum)
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
cv::Mat BpNetwork::cvtInputType(const Vdouble& in)
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

cv::Mat BpNetwork::cvtInputType(const Vec3d& in)
{
    Mat input(3, 1, CV_64FC1);

    for (int i = 0; i < 3;i++)
        input.at<double>(i) =
        2.0*(in[i] - InputMinMaxs[i][0]) / (InputMinMaxs[i][1] - InputMinMaxs[i][0]) - 1.0;
        
    return input;
}



void BpNetwork::tansig(Mat& x)
{
    auto it = x.begin<double>();
    auto itend = x.end<double>();

    while (it != itend)
    {
        *it = 2.0 / (1 + exp(-2.0 * *it)) - 1;
        it++;
    }
}

//--------------------end of Class `BpNetwork` -----------------------

