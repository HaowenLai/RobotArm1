/* *******************************************************************
 * This file is the realization of my work dealing with Aruco Markers.
 * Some functions relating to Aruco Markers are also contained.
** *******************************************************************/

#include "ArucoMarker.hpp"

using namespace cv;
using namespace std;

//-------------------- Definition of class `ArucoMarker`------------------
//public
ArucoMarker::ArucoMarker(
    vector<int>&& watchID,
    const cv::Mat cameraMat,
    const cv::Mat distCoeff,
    enum cv::aruco::PREDEFINED_DICTIONARY_NAME dn):
    watchMarkIds(watchID),
    cameraMatrix(cameraMat),
    distCoeffs(distCoeff),
    dict(aruco::getPredefinedDictionary(dn)),
    origin_rVecs(Vec3d(0,0,0)),
    origin_tVecs(Vec3d(0,0,0))
{ }

void ArucoMarker::calibrateOrigin(int calibMarkId)
{
    for(size_t i=0;i<markerIds.size();i++)
    {
        if(calibMarkId == markerIds[i])
        {
            origin_rVecs = rVecs[i];
            origin_tVecs = tVecs[i];
            break;
        }
    }
}

void ArucoMarker::detect(cv::Mat& img)
{
    markerIds.clear();
    aruco::detectMarkers(
        img,dict,
        markerCorners,
        markerIds,
        parameter.create(),
        rejectedCandidates,
        cameraMatrix,
        distCoeffs);
    aruco::drawDetectedMarkers(img,markerCorners,markerIds);
    aruco::estimatePoseSingleMarkers(
        markerCorners,
        20,
        cameraMatrix,
        distCoeffs,
        rVecs,tVecs);
}


void ArucoMarker::outputOffset(Mat& img, Point&& point)
{
    offset_rVecs.clear();
    offset_tVecs.clear();

    for(int watchMarkID : watchMarkIds)
    {
        for(size_t i = 0;i<markerIds.size();i++)
        {
            if(watchMarkID == markerIds[i])
            {
                offset_rVecs.push_back(rVecs[i] - origin_rVecs);
                offset_tVecs.push_back(tVecs[i] - origin_tVecs);
            }
        }
    }

    for(size_t i=0;i<offset_tVecs.size();i++)
    {        
        char temp[100];
        sprintf(temp,
            "tVec offset is: %+4.1f  %+4.1f  %+4.1f "
            "rVec offset is: %+4.1f  %+4.1f  %+4.1f",
            offset_tVecs[i][0],offset_tVecs[i][1],offset_tVecs[i][2],
            offset_rVecs[i][0],offset_rVecs[i][1],offset_rVecs[i][2]);

        //put text test
        putText(img,temp,Point(point.x,point.y+i*20),
                FONT_HERSHEY_PLAIN,1,Scalar(255,0,0),1);
    }
}

void ArucoMarker::outputOffset(bool clearConsole)
{
    offset_rVecs.clear();
    offset_tVecs.clear();

    for(int watchMarkID : watchMarkIds)
    {
        for(size_t i = 0;i<markerIds.size();i++)
        {
            if(watchMarkID == markerIds[i])
            {
                offset_rVecs.push_back(rVecs[i] - origin_rVecs);
                offset_tVecs.push_back(tVecs[i] - origin_tVecs);
            }
        }
    }

    if(clearConsole)
    for(size_t i=0;i<offset_tVecs.size();i++)
        printf("\033[1A\033[K\033[1A\033[K\033[1A\033[K\033[1A\033[K");
    
    for(size_t i=0;i<offset_tVecs.size();i++)
    {
        cout<<"No."<< i <<" rVec offset is:\n  "
            << setiosflags(ios::fixed|ios::left) << setprecision(4)
            << setw(12) << offset_rVecs[i][0] 
            << setw(12) << offset_rVecs[i][1]
            << setw(12) << offset_rVecs[i][2] <<endl;
        cout<<"No."<< i <<" tVec offset is:\n  " << setiosflags(ios::left)
            << setiosflags(ios::fixed|ios::left) << setprecision(4)
            << setw(12) << offset_tVecs[i][0]
            << setw(12) << offset_tVecs[i][1]
            << setw(12) << offset_tVecs[i][2] <<endl;
    }
}


//--------------------End definition of class `ArucoMarker`------------------



//to generate ArUco markers, just use once
void generateMarker(int id)
{
    Mat markerImg;
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    aruco::drawMarker(dict, id, 80, markerImg, 1);  //80 pixel

    namedWindow("haha", WINDOW_AUTOSIZE);
    imshow("haha", markerImg);
    waitKey(0);
}
