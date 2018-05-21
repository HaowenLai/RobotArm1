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
    int watchID,
    const cv::Mat cameraMat,
    const cv::Mat distCoeff,
    enum cv::aruco::PREDEFINED_DICTIONARY_NAME dn):
    specificMarkId(watchID),
    cameraMatrix(cameraMat),
    distCoeffs(distCoeff),
    specificMarkIndex(-1),
    origin_rVecs(Vec3d(0,0,0)),
    origin_tVecs(Vec3d(0,0,0)),
    dict(aruco::getPredefinedDictionary(dn))
{ }

void ArucoMarker::calibrateOrigin()
{
    if(specificMarkIndex >= 0)
    {
        origin_rVecs = rVecs[specificMarkIndex];
        origin_tVecs = tVecs[specificMarkIndex];
    }
}

void ArucoMarker::drawBoundaryAndAxis(cv::Mat& img)
{
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
        
    specificMarkIndex = -1;
    for(size_t i = 0;i<markerIds.size();++i)
    {
        aruco::drawAxis(img,cameraMatrix,distCoeffs,rVecs[i],tVecs[i],30);
        if(markerIds[i] == specificMarkId)
            specificMarkIndex = i;
    }
}

void ArucoMarker::outputOffset(bool clearLastResult)
{
    if(specificMarkIndex >= 0)
    {
        if(clearLastResult)
            printf("\033[1A\033[K\033[1A\033[K\033[1A\033[K\033[1A\033[K");
        
        cout<<"rVec offset is:\n  "
            << setiosflags(ios::fixed|ios::left) << setprecision(4)
            << setw(12) << rVecs[specificMarkIndex][0] - origin_rVecs[0] 
            << setw(12) << rVecs[specificMarkIndex][1] - origin_rVecs[1] 
            << setw(12) << rVecs[specificMarkIndex][2] - origin_rVecs[2] <<endl;
        cout<<"tVec offset is:\n  " << setiosflags(ios::left)
            << setiosflags(ios::fixed|ios::left) << setprecision(4)
            << setw(12) << tVecs[specificMarkIndex][0] - origin_tVecs[0]
            << setw(12) << tVecs[specificMarkIndex][1] - origin_tVecs[1]
            << setw(12) << tVecs[specificMarkIndex][2] - origin_tVecs[2] <<endl;
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
