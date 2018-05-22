#include "RsVideoCapture.hpp"
#include "ArucoMarker.hpp"

using namespace cv;
using namespace std;

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
    //camera instrinc matrix and distort coefficients
    const Mat ANC_cameraMatrix = (Mat_<double>(3, 3) 
        << 846.82,-0.2796, 328.36, 0, 849.605, 193.144, 0, 0, 1);
    const Mat ANC_distCoeffs = (Mat_<double>(1, 5)
        << 0.0699,0.091, -0.0072, 0.0036, -1.5246);
    const Mat RS_cameraMatrix = (Mat_<double>(3, 3)
        << 622.60,0, 312.12, 0, 623.37, 235.86, 0, 0, 1);
    const Mat RS_distCoeffs = (Mat_<double>(1, 4)
        << 0.156,-0.2792, 0, 0);

    Mat img_anc, img_rs;
    ArucoMarker ancMarkerId9(9, ANC_cameraMatrix, ANC_distCoeffs);
    ArucoMarker rsMarkerId9(9, RS_cameraMatrix, RS_distCoeffs);

    helpMsg();

    VideoCapture camera(3);
    RsVideoCapture camera_rs;
    namedWindow("ANC", WINDOW_AUTOSIZE);
    namedWindow("RS", WINDOW_AUTOSIZE);

    while (1)
    {
        camera >> img_anc;
        camera_rs >> img_rs;

        ancMarkerId9.drawBoundaryAndAxis(img_anc);
        rsMarkerId9.drawBoundaryAndAxis(img_rs);

        imshow("ANC", img_anc);
        imshow("RS", img_rs);

        ancMarkerId9.outputOffset(false);
        rsMarkerId9.outputOffset(false);

        switch ((char)waitKey(50))
        {
        case 'c':
            ancMarkerId9.calibrateOrigin();
            rsMarkerId9.calibrateOrigin();
            break;
        case 'q':
            return 0;
        default:
            break;
        }
    }

    return 0;
}
