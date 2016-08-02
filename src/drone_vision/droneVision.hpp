#ifndef DRONEVISION_HPP
#define DRONEVISION_HPP
#include <opencv2/opencv.hpp>
#include <string>
#include<sstream>
#include <iomanip>
/*****ROS Headers*********/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ardrone_autonomy/Navdata.h>
#include <crazy_landing/error_xy.h>
#include <crazy_landing/num_detected.h>
#include <crazy_landing/pid.h>
#include<crazy_landing/pic_direction.h>
/*****************************/
using namespace std;
using namespace cv;
/*****ROS convertion******/
class droneVision
{
private:
	ros::NodeHandle n;
  	 image_transport::ImageTransport it;
   	image_transport::Subscriber image_sub;
	//ros::Subscriber Navdata_sub;
	ros::Publisher DectetedNumber_pub;
	ros::Publisher error_pub;
	ros::Publisher num_pub;
	ros::Publisher pid_pub;
	ros::Publisher direction_pub;
	int kernel_size;
	//int reset=0;//reset param
	int Hmin,Hmax;//HSV threshold
	int Smin,Smax;
	int Vmin,Vmax;
	int areaThres;//contour area threshold
	int epsilon;
	bool RectDetected;
	bool isInside;
	float errorX,errorY;
	float altitude;//navdata
	float rotX;
	float rotY;
	float rotZ;//
	Mat ROI;
	Mat trainData;
	Mat responses;
	bool LEARN;
	bool DETECT;
    //float last_pub_time;
    //bool frame_lost;
	int trainNumber;
	int trainCount;
	int numberSize;
	int detectedNumber;
	int KNN_k;//k nearest
	int TNPS;// train number per sample
	int CNUM;//train class number 10
	CvKNearest knn;//KNN classifier
	//int P,I,D,kN;//pid param
	void windowsInit();
	void valueInit();//initiate default values
	void findYellowRect(const sensor_msgs::ImageConstPtr& msg);
	void detectNumber();
 	static void resetParam(int,void*);
 	static void learnCallback(int,void*);
 	static void detectCallback(int,void*);
 	void navdataCallback(const ardrone_autonomy::Navdata& msg);
 	vector<float> distance_cal(float , float, float, float,float, float);
public:
	droneVision():it(n)
  {
	valueInit();
  	windowsInit();
	image_sub = it.subscribe("/ardrone/image_raw", 1,&droneVision::findYellowRect, this);
	//Navdata_sub=n.subscribe("/ardrone/navdata",1,&droneVision::navdataCallback,this);
	error_pub=n.advertise<crazy_landing::error_xy>("error_xy",1);
	num_pub=n.advertise<crazy_landing::num_detected>("num_detected",1);
	//pid_pub=n.advertise<crazy_landing::pid>("pid",1);
	direction_pub=n.advertise<crazy_landing::pic_direction>("pic_direction",1);


  }
  
};
#endif
