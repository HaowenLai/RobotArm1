#include <opencv2/opencv.hpp>
#include <string>
#include <math.h>

using namespace std;
using namespace cv;

//const value definition
const string WIN_NAME("windows");
const float WIDTH = 1024;
const float HEIGHT= 768;
const Point2f origin(300,HEIGHT-30);
const float ARM1 = 200; //arm 1,200 pixels
const float ARM2 = 200; //arm 2,200 pixels

inline void dispCvt2Cal(Point2f& src,Point2f& dst)
{
    dst.x = src.x - origin.x;
    dst.y = origin.y - src.y;
}

inline void calCvt2Disp(Point2f& src,Point2f& dst)
{
    dst.x = src.x + origin.x;
    dst.y = origin.y - src.y;
}

void calJointAngle(vector<float>& jointsAngle, Point2f& target)
{
    Point2f dst(.0f, .0f);
    dispCvt2Cal(target,dst); //convert pixel locatoin to calculate location

    float l = sqrt(dst.x*dst.x + dst.y*dst.y);  //length between dst and the origin
    float angle1 = acos((ARM1*ARM1+l*l-ARM2*ARM2)/(2*ARM1*l)); //ARM1 and l
    float angle2 = acos((ARM2*ARM2+l*l-ARM1*ARM1)/(2*ARM2*l)); //ARM2 and l
    float gamma  = acos(dst.x/l);   //l and line parallel to x axis
    float alpha  = angle1 + gamma;  //ARM1 and line parallel to x axis
    float beta   = gamma - angle2;  //ARM2 and line parallel to x axis

    jointsAngle.push_back(alpha);
    jointsAngle.push_back(beta);
}

//controlled variable is angle
void PidControl(vector<float>& jointPre,const vector<float>& jointDst)
{
    //PID parameter
    const float Kp = 0.1;

    float error0 = jointDst[0]-jointPre[0];
    float error1 = jointDst[1]-jointPre[1];
    jointPre[0] += (error0*Kp)>CV_PI/12? CV_PI/12 : error0*Kp;
    jointPre[1] += (error1*Kp)>CV_PI/12? CV_PI/12 : error1*Kp;
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

        Point2f target(x,y);
        circle(img,target,4,Scalar(70,0,70),-1); //draw the destinition point

        //vector<float>* jointsPrev = (vector<float>*)param; //previous joints angle
        static vector<float> jointsPrev(2);
        vector<float> jointsDst;                           //destiny joints angle
        
        //control simulation
        calJointAngle(jointsDst,target);
        do
        {           
            PidControl(jointsPrev,jointsDst);

            //angle to location
            Point2f joint1(ARM1*cos(jointsPrev[0]),ARM1*sin(jointsPrev[0]));
            Point2f joint2(joint1.x+ARM2*cos(jointsPrev[1]),joint1.y+ARM2*sin(jointsPrev[1]));
            calCvt2Disp(joint1,joint1);
            calCvt2Disp(joint2,joint2);

            //draw arms
            line(img,origin,joint1,Scalar(0,255,0),2);
            line(img,joint1,joint2,Scalar(255,0,0),2);
            imshow(WIN_NAME,img);
            waitKey(100);
        }while(abs(jointsDst[0]-jointsPrev[0])>0.03 || 
               abs(jointsDst[1]-jointsPrev[1])>0.03); 
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