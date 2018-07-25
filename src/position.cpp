/* *************************************************************
 *                 CXX file for position.cpp
 *   Class to get position.
 *   There two classes, namely `ArucoMarker` and `CubePosition`.
 * Class `ArucoMarker` deals with the marker positon, whose unit
 * is mm in real world coordinate. Class `CubePosition` to deal 
 * with the position of cubes without markers by finding its
 * contour. Its unit is pixel in image coordinate.
***************************************************************/

#include "position.hpp"
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


float ArucoMarker::angle(int id)
{
    int idIndex = 0;
    for(size_t i=0;i<markerIds.size();i++)
    {
        if(markerIds[i] == id)
        {
            idIndex = i;
            break;
        }
    }
    
    auto pt1 = markerCorners[idIndex][0];
    auto pt2 = markerCorners[idIndex][1];
    
    return atan((pt2.y-pt1.y)/(pt1.x-pt2.x));
}


cv::Point2f ArucoMarker::firstCorner(int id)
{
    int idIndex = 0;
    for(size_t i=0;i<markerIds.size();i++)
    {
        if(markerIds[i] == id)
        {
            idIndex = i;
            break;
        }
    }
    return markerCorners[idIndex][0];
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





//-------------------- Definition of class `CubePosition`------------------
//public
CubePosition::CubePosition(Rect detectArea,
                          double validLenMax,
                          double validLenMin):
    detectArea(detectArea),
    validPerimeterMax(validLenMax),
    validPerimeterMin(validLenMin),
    cubeExistFlag(false),
    frameStamp(0)
{}

float CubePosition::angle()
{
    Point2f vertex[4];
    cubeBox.points(vertex);
    
    int upIndex[2] {0,1};   //left,right,right is min
    if(vertex[upIndex[0]].y < vertex[upIndex[1]].y)
    {
        upIndex[0] = 1;
        upIndex[1] = 0;
    }
    for(int i=2;i<4;i++)
    {
        if(vertex[i].y < vertex[upIndex[1]].y)
        {
            upIndex[0] = upIndex[1];
            upIndex[1] = i;
        }
        else if(vertex[i].y < vertex[upIndex[0]].y)
        {
            upIndex[0] = i;
        }
    }

    return atan((vertex[upIndex[0]].y-vertex[upIndex[1]].y)/
                (vertex[upIndex[1]].x-vertex[upIndex[0]].x));
}

bool CubePosition::cubeExist()
{
    return cubeExistFlag;
}

void CubePosition::detect(Mat& img)
{
    cubeExistFlag = false;
    
    Mat roi = img(detectArea);
    Mat grayImg,binImg;

    cvtColor(roi,grayImg,COLOR_BGR2GRAY);
    GaussianBlur(grayImg,grayImg,Size(7,7),0.8);
    //GaussianBlur(grayImg,grayImg,Size(7,7),0.5);
    //blur(grayImg,grayImg,Size(12,12));
    threshold(grayImg,binImg,0,255,THRESH_OTSU); //binaryzation

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binImg,contours,hierarchy,RETR_CCOMP,CHAIN_APPROX_SIMPLE);

    //check contour validation
    for(size_t i=0; i<contours.size(); i++)
    {
        double length = arcLength(contours[i],true);
        if(length > validPerimeterMax || length < validPerimeterMin)
        {
            continue;
        }

        cubeBox = minAreaRect(contours[i]);
        
        cubeExistFlag = true;
        break;
    }

    frameStamp++;
}


void CubePosition::drawBoundry(cv::Mat& img)
{
    if(!cubeExistFlag)
        return;

    Point2f vertex[4];
    cubeBox.points(vertex);

    //add the offset caused by roi
    for(int i =0;i<4;i++)
    {
        vertex[i].x += detectArea.x;
        vertex[i].y += detectArea.y;
    }

    //draw the min rectangle that encloses the cube
    line(img,vertex[0],vertex[1],Scalar(100,200,211),2,LINE_AA);
	line(img,vertex[1],vertex[2],Scalar(100,200,211),2,LINE_AA);
	line(img,vertex[2],vertex[3],Scalar(100,200,211),2,LINE_AA);
	line(img,vertex[3],vertex[0],Scalar(100,200,211),2,LINE_AA);

    //draw center
    Point s1,l,r,u,d;   //center,left,right,up,down
	s1.x=(vertex[0].x+vertex[2].x)/2.0;
	s1.y=(vertex[0].y+vertex[2].y)/2.0;
	//
    l.x=s1.x-10;
	l.y=s1.y;
    //
	r.x=s1.x+10;
	r.y=s1.y;
    //
	u.x=s1.x;
	u.y=s1.y-10;
    //
	d.x=s1.x;
	d.y=s1.y+10;
	line(img,l,r,Scalar(100,200,211),2,LINE_AA);
	line(img,u,d,Scalar(100,200,211),2,LINE_AA);
}


Point2f CubePosition::firstCorner()
{
    Point2f vertex[4];
    cubeBox.points(vertex);
    
    int upIndex[2] {0,1};   //left,right,right is min
    if(vertex[upIndex[0]].y < vertex[upIndex[1]].y)
    {
        upIndex[0] = 1;
        upIndex[1] = 0;
    }
    for(int i=2;i<4;i++)
    {
        if(vertex[i].y < vertex[upIndex[1]].y)
        {
            upIndex[0] = upIndex[1];
            upIndex[1] = i;
        }
        else if(vertex[i].y < vertex[upIndex[0]].y)
        {
            upIndex[0] = i;
        }
    }

    if(vertex[upIndex[1]].x > vertex[upIndex[0]].x)
    {
        vertex[upIndex[1]].x += detectArea.x;
        vertex[upIndex[1]].y += detectArea.y;
        return vertex[upIndex[1]];
    }
    else
    {
        vertex[upIndex[0]].x += detectArea.x;
        vertex[upIndex[0]].y += detectArea.y;
        return vertex[upIndex[0]];
    }
}


Point2f CubePosition::getPosition()
{
    auto center = cubeBox.center;

    Point2f tmp;
    tmp.x = center.x + detectArea.x;
    tmp.y = center.y + detectArea.y;

    return tmp;
}


bool CubePosition::isNewFrame()
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

void CubePosition::outputPos(cv::Mat& img,cv::Point&& startPoint)
{
    if(!cubeExistFlag)
        return;
    
    char temp[100];
    sprintf(temp,
        "pixel coordinate: (%+4.1f, %+4.1f)",
        cubeBox.center.x + detectArea.x,
        cubeBox.center.y + detectArea.y);
    //put text test
    putText(img,temp,Point(startPoint.x,startPoint.y),
            FONT_HERSHEY_PLAIN,1,Scalar(255,0,0),1);
    
}

void CubePosition::outputPos(bool clearConsole)
{
    if(!cubeExistFlag)
        return;

    if(clearConsole)
        printf("\033[1A\033[K");
    
    cout<<"pixel coordinate is:\n  "
            << setiosflags(ios::fixed|ios::left) << setprecision(4)
            << setw(12) << cubeBox.center.x + detectArea.x
            << setw(12) << cubeBox.center.y + detectArea.y <<endl;
}

//--------------------End definition of class `CubePosition`------------------


cv::Vec3d upperPC2frontAC(cv::Point2f upperPC)
{
    const float p1 = -0.7351;
    const float p2 = 369.2;
    Vec3d frontAC(0.,62.0,0.);

    frontAC[0] = p1*upperPC.x + p2;
    return frontAC;
}


cv::Vec3d upperPC2upperAC(cv::Point2f upperPC)
{
    const float xp1 = 0.7259;
    const float xp2 = -230.4;
    const float yp1 = 0.7018;
    const float yp2 = -175.2;
    Vec3d upperAC(0.,0.,0.);

    upperAC[0] = xp1*upperPC.x + xp2;
    upperAC[1] = yp1*upperPC.y + yp2;
    return upperAC;
}

