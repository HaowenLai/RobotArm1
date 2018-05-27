#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

//const value definition
const char* WIN_NAME = "windows";
const float WIDTH = 1024;
const float HEIGHT= 768;
const Point2f origin(300,HEIGHT-30);
const float ARM1 = 200; //arm 1,200 pixels
const float ARM2 = 200; //arm 2,200 pixels


void dispCvt2Cal(Point2f& src,Point2f& dst)
{
    dst.x = src.x - origin.x;
    dst.y = origin.y - src.y;
}

void calCvt2Disp(Point2f& src,Point2f& dst)
{
    dst.x = src.x + origin.x;
    dst.y = origin.y - src.y;
}

void calJointPoints(vector<Point2f>& joints, Point2f& target)
{
    Point2f joint1(.0f, .0f);
    Point2f joint2(.0f, .0f);
    Point2f dst(.0f, .0f);
    dispCvt2Cal(target,dst);

    float l = sqrt(dst.x*dst.x + dst.y*dst.y);  //length between dst and the origin
    float angle1 = acos((ARM1*ARM1+l*l-ARM2*ARM2)/(2*ARM1*l)); //ARM1 and l
    float angle2 = acos((ARM2*ARM2+l*l-ARM1*ARM1)/(2*ARM2*l)); //ARM2 and l
    float gamma  = acos(dst.x/l);   //l and line parallel to x axis
    float alpha  = angle1 + gamma;  //ARM1 and l
    float beta   = gamma - angle2;  //ARM2 and l

    joint1.x=ARM1*cos(alpha);
    joint1.y=ARM1*sin(alpha);
    joint2.x=joint1.x+ARM2*cos(beta);
    joint2.y=joint1.y+ARM2*sin(beta);

    calCvt2Disp(joint1,joint1);
    calCvt2Disp(joint2,joint2);
    joints.push_back(joint1);
    joints.push_back(joint2);
}


static void onMouse(int event, int x, int y, int flags, void* param)
{
	if( event == EVENT_LBUTTONDOWN )
	{
		Mat img(HEIGHT,WIDTH,CV_32FC3,Scalar(255,255,255));
        circle(img,origin,ARM1+ARM2,Scalar(0,0,255));               //draw max circle
        circle(img,origin,4,Scalar(0,0,255),-1);                    //draw origin
        line(img,origin,Point(origin.x,20),Scalar(0,0,255));        //y-axis
        line(img,origin,Point(WIDTH-20,origin.y),Scalar(0,0,255));  //x-axis

        Point2f target(x,y);    //the destinition point
        circle(img,target,4,Scalar(70,0,70),-1);

        vector<Point2f> joints;
        calJointPoints(joints,target);

        //draw arms
        line(img,origin,joints[0],Scalar(0,255,0),2);
        line(img,joints[0],joints[1],Scalar(255,0,0),2);

        imshow(WIN_NAME,img);
	}
}

int main()
{  
    Mat img(HEIGHT,WIDTH,CV_32FC3,Scalar(255,255,255));
    circle(img,origin,ARM1+ARM2,Scalar(0,0,255));   //draw max circle
    circle(img,origin,4,Scalar(0,0,255),-1);        //draw origin
    line(img,origin,Point(origin.x,20),Scalar(0,0,255));        //y-axis
    line(img,origin,Point(WIDTH-20,origin.y),Scalar(0,0,255));  //x-axis
    namedWindow(WIN_NAME,WINDOW_AUTOSIZE);
    setMouseCallback(WIN_NAME,onMouse);
    imshow(WIN_NAME,img);

    while(1)
    {
        switch ((char)waitKey(0))
        {
        case 'q':
            return 0;
        default:
            break;
        }
    }
    return 0;
}