/* *******************************************************************
 * This file is the realization of my work dealing with Aruco Markers.
 * Some functions relating to Aruco Markers are also contained.
** *******************************************************************/

#include "ArucoMarker.hpp"
#include <cmath>

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
    origin_tVecs(Vec3d(0,0,0)),
    frameStamp(0)
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
    foundIds.clear();
    offset_rVecs.clear();
    offset_tVecs.clear();

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
    
    for(int watchMarkID : watchMarkIds)
    {
        for(size_t i = 0;i<markerIds.size();i++)
        {
            if(watchMarkID == markerIds[i])
            {
                foundIds.push_back(watchMarkID);
                offset_rVecs.push_back(rVecs[i] - origin_rVecs);
                offset_tVecs.push_back(tVecs[i] - origin_tVecs);
            }
        }
    }
    frameStamp++;
}


float ArucoMarker::angle(int idIndex)
{
    auto pt1 = markerCorners[idIndex][0];
    auto pt2 = markerCorners[idIndex][3];
    
    return atan((pt1.y-pt2.y)/(pt2.x-pt1.x));
}


bool ArucoMarker::isNewFrame()
{
    static int fs = 0;
    
    if(fs == frameStamp)
        return false;
    else
    {
        fs = frameStamp;
        return true;
    }
}

int ArucoMarker::index(int id)
{
    for(size_t i=0;i<foundIds.size();i++)
    {
        if(id == foundIds[i])
            return i;
    }
    return -1;
}

void ArucoMarker::outputOffset(Mat& img, Point&& point)
{
    for(size_t i=0;i<offset_tVecs.size();i++)
    {        
        char temp[100];
        sprintf(temp,
            "tVec is: %+4.1f  %+4.1f  %+4.1f "
            "rVec is: %+4.1f  %+4.1f  %+4.1f",
            offset_tVecs[i][0],offset_tVecs[i][1],offset_tVecs[i][2],
            offset_rVecs[i][0],offset_rVecs[i][1],offset_rVecs[i][2]);

        //put text test
        putText(img,temp,Point(point.x,point.y+i*20),
                FONT_HERSHEY_PLAIN,1,Scalar(255,0,0),1);
    }
}

void ArucoMarker::outputOffset(bool clearConsole)
{
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
