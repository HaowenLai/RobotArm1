#include "rsDev.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <librealsense2/rs.hpp>

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
    Mat img_anc,img_rs;
    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners, rejectedCandidates;
    aruco::DetectorParameters parameter;
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

    //rotate and translation vectors
    vector<Vec3d> rVecs,tVecs;
    //to record the original position info
    Vec3d origin_rVecs = Vec3d(0,0,0),origin_tVecs = Vec3d(0,0,0);
    
    const int specificMarkId = 3;       //only observe specific id
    
    const Mat ANC_cameraMatrix = (Mat_<double>(3,3) 
        << 846.82,-0.2796,328.36,0,849.605,193.144,0,0,1);
    const Mat ANC_distCoeffs = (Mat_<double>(1,5) 
        << 0.0699,0.091,-0.0072,0.0036,-1.5246);
    const Mat RS_cameraMatrix = (Mat_<double>(3,3) 
        << 622.60,0,312.12,0,623.37,235.86,0,0,1);
    const Mat RS_distCoeffs = (Mat_<double>(1,4) 
        << 0.156,-0.2792,0,0);

    helpMsg();

    VideoCapture camera(3);
    RsVideoCapture camera_rs;
    namedWindow("ANC", WINDOW_AUTOSIZE);
    namedWindow("RS", WINDOW_AUTOSIZE);
    
    while (1)
    {
        camera    >> img_anc;
        camera_rs >> img_rs;

        aruco::detectMarkers(
            img_rs,dict,markerCorners,markerIds,parameter.create(),rejectedCandidates);
        aruco::drawDetectedMarkers(img_rs,markerCorners,markerIds);
        aruco::estimatePoseSingleMarkers(
            markerCorners,20,RS_cameraMatrix,RS_distCoeffs,rVecs,tVecs);
        
        int specificMarkIndex = -1;
        for(size_t i = 0;i<markerIds.size();i++)
        {
            aruco::drawAxis(img_rs,RS_cameraMatrix,RS_distCoeffs,rVecs[i],tVecs[i],30);
            if(markerIds[i] == specificMarkId)
                specificMarkIndex = i;
        }

        imshow("ANC",img_anc);
        imshow("RS",img_rs);
        if(specificMarkIndex >= 0)
        {
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
