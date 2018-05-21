//#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <librealsense2/rs.hpp>
#include "rsDev.h"

using namespace cv;
using namespace std;

//to generate ArUco markers, just use once
inline void generateMarker(int id)
{
    Mat markerImg;
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    aruco::drawMarker(dict, id, 80, markerImg, 1);

    namedWindow("haha", WINDOW_AUTOSIZE);
    imshow("haha", markerImg);
    waitKey(0);
}

//help message
inline void helpMsg()
{
    cout << "help messages for this program\n"
            "key 'c' : to calibrate all markers original positions\n"
            "key 'q' : quit the program\n"
            "\n---press <Enter> to continue---";
    cin.get();
    printf("\n\n\n\n\n");
}



//detect ArUco markers and estimate pose
int main()
{
    Mat img;
    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameter;
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

    //rotate and translation vectors
    vector<Vec3d> rVecs,tVecs;
    //to record the original position info
    Vec3d origin_rVecs = Vec3d(0,0,0),origin_tVecs = Vec3d(0,0,0);
    
    const int specificMarkId = 3;       //only observe specific id
    const Mat cameraMatrix = (Mat_<double>(3,3) 
        << 846.82,-0.2796,328.36,0,849.605,193.144,0,0,1);
    const Mat distCoeffs = (Mat_<double>(1,5) 
        << 0.0699,0.091,-0.0072,0.0036,-1.5246);

    helpMsg();
    /*
    VideoCapture camera(0);
    if (!camera.isOpened())
    {
        cout << "cannot open camera!\n";
        return -1;
    }
    */
    RsVideoCapture camera;

    //rs2::pipeline pipe;
    //configRealsense(pipe);
    namedWindow("haha", WINDOW_AUTOSIZE);

    while (1)
    {
        camera >> img;
        //getRsMat(pipe,img);
        aruco::detectMarkers(
            img,dict,markerCorners,markerIds,parameter.create(),rejectedCandidates);
        aruco::drawDetectedMarkers(img,markerCorners,markerIds);
        aruco::estimatePoseSingleMarkers(
            markerCorners,20,cameraMatrix,distCoeffs,rVecs,tVecs);
        
        int specificMarkIndex = -1;
        for(size_t i = 0;i<markerIds.size();i++)
        {
            aruco::drawAxis(img,cameraMatrix,distCoeffs,rVecs[i],tVecs[i],30);
            if(markerIds[i] == specificMarkId)
                specificMarkIndex = i;
        }

        imshow("haha",img);
        if(rVecs.size()>0)
        {
            printf("\033[1A\033[K\033[1A\033[K\033[1A\033[K\033[1A\033[K");
            cout<<"rVec offset is:\n  "
                << rVecs[specificMarkIndex][0] - origin_rVecs[0]<<" "
                << rVecs[specificMarkIndex][1] - origin_rVecs[1]<<" "
                << rVecs[specificMarkIndex][2] - origin_rVecs[2]<<" "<<endl;
            cout<<"tVec offset is:\n  "
                << tVecs[specificMarkIndex][0] - origin_tVecs[0]<<" "
                << tVecs[specificMarkIndex][1] - origin_tVecs[1]<<" "
                << tVecs[specificMarkIndex][2] - origin_tVecs[2]<<" "<<endl;
        }
        
        switch((char)waitKey(50))
        {
        case 'c':
            if(specificMarkIndex >= 0)
            {
                origin_rVecs = rVecs[specificMarkIndex];
                origin_tVecs = tVecs[specificMarkIndex];
            }
            break;
        case 'q':
            return 0;
        default:
            break;
        }
    }

    return 0;
}
