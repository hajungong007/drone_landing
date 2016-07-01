#include"droneVision.hpp"
#include<math.h>
#define l 80 //distance from camera to the center of drone
#define alpha 1.45  //view width X of camera  ? degree need to be measured
#define beta 0.93 //view width Y of camera  ? degree to be measured

//image 640x360
//errorX[-320,+320],errorY[-180,+180]
//rotZ=rotZ-rotZ0

vector<float> droneVision::distance_cal(float rotX, float rotY, float rotZ, float altd,float errorX, float errorY)
{
    vector<float> distance;
    float dist_x1,dist_x2,dist_y1,dist_y2;
    float dist_x,dist_y;
    float m,n;  //length and width of image
    rotX=rotX/180*3.14;
    rotY=rotY/180*3.14;
    rotZ=(rotZ-140)/180*3.14;
    m=altd*(tan(rotX+alpha/2)-tan(rotX-alpha/2));
    n=altd*(tan(rotY+beta/2)-tan(rotY-beta/2));
    dist_x1=-(altd*(tan(rotY+beta/2)+tan(rotY-beta/2))/2+l/cos(rotY))*sin(rotZ);
    dist_y1=altd*(tan(rotX+alpha/2)+tan(rotX-alpha/2))/2*cos(rotZ)-l*sin(rotZ);
    dist_x2=n*errorY/180*cos(rotZ)+(-m*errorX/320)*sin(rotZ);
    dist_y2=(-m*errorX/320)*cos(rotZ)-n*errorY/180*sin(rotZ);
    dist_x=dist_x1+dist_x2;
    dist_y=dist_y1+dist_y2;
    distance.push_back(dist_x);
    distance.push_back(dist_y);
    return distance;//mm
}

