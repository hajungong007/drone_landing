#include "droneVision.hpp"
#include <math.h>
void droneVision::findYellowRect(const sensor_msgs::ImageConstPtr& msg){
/***************ROS convertion*************/
cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
/***********************************************/
	//VideoCapture vCap(0);
	//Mat image_bgr=imread(argv[1],CV_LOAD_IMAGE_COLOR);
	Mat img_bgr;
	Mat img_hsv;
	Mat img_threshold;
	Mat img_erode;
	Mat img_dilate;
	Mat element;//for erode and dilate
	string str;
	stringstream ss;

	//vCap>>img_bgr;
	img_bgr=cv_ptr->image;
	int COLS=img_bgr.cols;
	int ROWS=img_bgr.rows;
	//while(!img_bgr.empty()){
	cvtColor(img_bgr,img_hsv,CV_BGR2HSV);//convert to HSV image
	inRange(img_hsv,Scalar(Hmin,Smin,Vmin),Scalar(Hmax,Smax,Vmax),img_threshold);//threshold to yellow
	element = getStructuringElement(MORPH_RECT,Size(2*kernel_size+1,2*kernel_size+1),Point(kernel_size,kernel_size));
	dilate(img_threshold,img_dilate,element);//local ax
	erode(img_dilate,img_erode,element);//local min
	/*find contours*/
	Mat img_contour=img_erode.clone();//copy image to do findContours, because the src image will be modified
	vector<vector<Point> > contours;
	//vector<vector4i> hierarchy;
	findContours(img_contour,contours,/*hierarchy,*/CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

	//find the contour with the largest area
	double area;
	double maxArea=0;
	maxArea=0;
	int maxIdx=-1;
	for(int idx=0;idx<contours.size();idx++){
		area=contourArea(contours[idx]);
		if(area>=areaThres){
			if(maxArea<area) {maxArea=area; maxIdx=idx;}
		}
	}
 			//Use approxPolyDP to simplifie the contour and give the four vertices
		if(maxIdx>=0){

			/*determine if the contour touch the board*/
			for(int i=0;i<contours[maxIdx].size();i++){
				if((COLS-contours[maxIdx][i].x<=2) || (contours[maxIdx][i].x<=2) || (ROWS-contours[maxIdx][i].y<=2) || (contours[maxIdx][i].y<=2)){
					isInside=0;
					break;
				}
				isInside=1;
			}
			/**************************************************/
			vector<Point> hull;
			convexHull(contours[maxIdx],hull);//find convex hull of largest contour

			vector<vector<Point> > targetRect(0);
			vector<Point> approxVertices;
			Moments mu;//to calculate the barycenter
			Point2f mc;
			/*if the contour touch the board, just draw hull*/
			if(!isInside){
				targetRect.push_back(hull);
				drawContours(img_bgr,targetRect,0,Scalar(0,0,255),1,8);
					//arrow
 					mu = moments( hull, false );
					///  Get the mass centers:
 					mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
 					RectDetected=false;
			}
			/******************************************************/
			/*if the contour is inside, do approxPolyDP to take the four vertices and draw*/
			else{
				approxPolyDP(hull,approxVertices,epsilon,true);
				if(approxVertices.size()==4) {  //if four vertices
					targetRect.push_back(approxVertices);
					drawContours(img_bgr,targetRect,0,Scalar(0,0,255),1,8);
					//arrow
 						mu = moments( approxVertices, false );
						///  Get the mass centers:
 						mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
 						RectDetected=true;
 					}else{  //if not four vertices draw hull
 						targetRect.push_back(hull);
						drawContours(img_bgr,targetRect,0,Scalar(0,0,255),1,8);
						//arrow
 						mu = moments( hull, false );
						///  Get the mass centers:
 						mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
 						RectDetected=false;
					}
			}
 			/***************get a normolized birdview patch of the rectangular*******************/
 			 	if(RectDetected){
 			 		Point2f Dst[4]={Point2f(0,0),Point2f(numberSize+1,0),Point2f(numberSize+1,numberSize+1),Point2f(0,numberSize+1)};//extract two more pixel larger for latter removal
 			 			int topVertice=0;
 			 			int topY=approxVertices[0].y;
 			 			for(int i=1;i<4;i++){
 			 				if(approxVertices[i].y<topY){
 			 					 topVertice=i;
 			 					 topY=approxVertices[i].y;
 			 				 }
 			 			}
 			 			int secTopVertice=(topVertice+1)%4;
 			 			/*if(( (approxVertices[(topVertice-1+4)%4].y-approxVertices[topVertice].y)*(approxVertices[(topVertice-1+4)%4].y-approxVertices[topVertice].y)
 			 				+(approxVertices[(topVertice-1+4)%4].x-approxVertices[topVertice].x)*(approxVertices[(topVertice-1+4)%4].x-approxVertices[topVertice].x))
 			 				<((approxVertices[(topVertice+1)%4].y-approxVertices[topVertice].y)*(approxVertices[(topVertice+1)%4].y-approxVertices[topVertice].y)
 			 				+(approxVertices[(topVertice+1)%4].x-approxVertices[topVertice].x)*(approxVertices[(topVertice+1)%4].x-approxVertices[topVertice].x)))*/
 			 			if(approxVertices[(topVertice-1+4)%4].y<approxVertices[secTopVertice].y)
 			 			{

 			 			secTopVertice=(topVertice-1+4)%4;//take another short exteme of the topVertice

 			 			}
 			 			int v0,v1,v2,v3;
 			 			if(approxVertices[secTopVertice].x<approxVertices[topVertice].x){
 			 				v0=secTopVertice;
 			 				v1=topVertice;
 			 			}else{
 			 				v0=topVertice;
 			 				v1=secTopVertice;
 			 			}
 			 			if(((v1-v0)==1)||((v1-v0)==-3)){
 			 				v2=(v1+1)%4;
 			 				v3=(v1+2)%4;
 			 			}
 			 			else{
 			 				v2=(v1-1+4)%4;
 			 				v3=(v1-2+4)%4;
 			 			}
 						//cout<<approxVertices[v0]<<approxVertices[v1]<<approxVertices[v2]<<approxVertices[v3]<<endl;
						Point2f Src[4]={approxVertices[v0],approxVertices[v1],approxVertices[v2],approxVertices[v3]};
						//cout<<approxVertices<<endl;
						Mat M=getPerspectiveTransform(Src,Dst);
						warpPerspective(img_erode,ROI,M, Size(numberSize,numberSize));
						ROI=ROI(Range(1,numberSize-1),Range(1,numberSize-1));//remove noisy board !!Range is exclusive at right
						/*Calculate direction*/
						crazy_landing::pic_direction dirmsg;
						dirmsg.header.stamp=ros::Time::now();
						dirmsg.err_angle=atan(float(approxVertices[v0].x-approxVertices[v3].x+approxVertices[v1].x-approxVertices[v2].x)/(approxVertices[v0].y-approxVertices[v3].y+approxVertices[v1].y-approxVertices[v2].y));
						dirmsg.is_direction=true;
						direction_pub.publish(dirmsg);
						detectNumber();//Do detection
						
 				}

			//arrowedLine(img_bgr,Point(COLS/2,ROWS/2),mc,Scalar(0,255,0),2);
			errorX=-(mc.y-ROWS/2);
			errorY=-(mc.x-COLS/2);
		/*transform and publish*/
			crazy_landing::error_xy errmsg;
			//vector<float> Err=distance_cal(rotX, rotY, rotZ, altitude,errorX, errorY);
			errmsg.header.stamp=ros::Time::now();
			errmsg.err_x=errorX;  //Err[0];
			errmsg.err_y=errorY; //Err[1];
			errmsg.is_target_detected=true;
			error_pub.publish(errmsg);


			if(RectDetected){
					/*display errorVector*/
					ss.clear();
					ss<<"Detected!------"<<'['<<setw(4)<<errorX<<','<<setw(4)<<errorY<<']';
					ss>>str;
					displayOverlay("MainWindow",str,100);
					/************************/
			}else{
				ss.clear();
					ss<<"Not-detected!--"<<'['<<setw(4)<<errorX<<','<<setw(4)<<errorY<<']';
					ss>>str;
					displayOverlay("MainWindow",str,100);
			}
	}
	else {
			RectDetected=false;
			displayOverlay("MainWindow","No contour found!",100);//cout<<"No contour found!"<<endl;
			crazy_landing::error_xy errmsg;
			errmsg.header.stamp=ros::Time::now();
			errmsg.err_x=-1;//publish error_xy
			errmsg.err_y=-1;
			errmsg.is_target_detected=false;
			error_pub.publish(errmsg);//
	}

	if(!RectDetected){//publish num_detected
		crazy_landing::num_detected nummsg;
		nummsg.header.stamp=ros::Time::now();
		nummsg.num=-1;
		nummsg.is_detected=false;
		num_pub.publish(nummsg);
		
		
		crazy_landing::pic_direction dirmsg;
		dirmsg.header.stamp=ros::Time::now();
	   dirmsg.err_angle=0;
		dirmsg.is_direction=false;
		direction_pub.publish(dirmsg);
	}//
//pid publish
/*
	crazy_landing::pid pidmsg;
	pidmsg.P=P;
	pidmsg.I=I;
	pidmsg.D=D;
	pidmsg.kN=kN;
	pid_pub.publish(pidmsg);*/
/***********************************************************************************************************/
	/*****************/
	imshow("MainWindow",img_bgr);
	//imshow("HSV",img_hsv);
	//imshow("Threshold",img_threshold);
	//imshow("Dilation",img_dilate);
	imshow("Threshold&Dilation&Erosion",img_erode);
	//if(c=='q') break;
	//vCap>>img_bgr;
	//}
waitKey(20);
return ;
}

