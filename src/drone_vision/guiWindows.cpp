#include "droneVision.hpp"
void droneVision::windowsInit(){
	//namedWindow("HSV",CV_WINDOW_AUTOSIZE);
	namedWindow("MainWindow",CV_WINDOW_AUTOSIZE | CV_GUI_EXPANDED);
	//namedWindow("Threshold",CV_WINDOW_AUTOSIZE);
	//namedWindow("Dilation",CV_WINDOW_AUTOSIZE);
	//namedWindow("Threshold&Dilation&Erosion",CV_WINDOW_AUTOSIZE);
	//namedWindow("DetectedNumber",CV_WINDOW_NORMAL);
	//namedWindow("Param Control",CV_WINDOW_AUTOSIZE);
	//createTrackbar("Reset","Param Control",&reset,1,resetParam);
	cvCreateTrackbar("H min",NULL,&Hmin,255,NULL);
	cvCreateTrackbar("H max",NULL,&Hmax,255,NULL);
	cvCreateTrackbar("S min",NULL,&Smin,255,NULL);
	cvCreateTrackbar("S max",NULL,&Smax,255,NULL);
	cvCreateTrackbar("V min",NULL,&Vmin,255,NULL);
	cvCreateTrackbar("V max",NULL,&Vmax,255,NULL);
	cvCreateTrackbar("KernelSize",NULL,&kernel_size,5,NULL);
	cvCreateTrackbar("Area x100",NULL,&areaThres,1000,NULL);
	cvCreateTrackbar("Approx eps",NULL,&epsilon,60,NULL);
	createButton("Reset",resetParam,this,CV_PUSH_BUTTON);
	cvCreateTrackbar("KNN_k",NULL,&KNN_k,32,NULL);
	createButton("L0",learnCallback,this,CV_CHECKBOX,0);
	createButton("L1",learnCallback,this,CV_CHECKBOX,0);
	createButton("L2",learnCallback,this,CV_CHECKBOX,0);
	createButton("L3",learnCallback,this,CV_CHECKBOX,0);
	createButton("L4",learnCallback,this,CV_CHECKBOX,0);
	createButton("L5",learnCallback,this,CV_CHECKBOX,0);
	createButton("L6",learnCallback,this,CV_CHECKBOX,0);
	createButton("L7",learnCallback,this,CV_CHECKBOX,0);
	createButton("L8",learnCallback,this,CV_CHECKBOX,0);
	createButton("L9",learnCallback,this,CV_CHECKBOX,0);
	createButton("Detect",detectCallback,this,CV_CHECKBOX,0);
	//cvCreateTrackbar("P",NULL,&P,8,NULL);
	//cvCreateTrackbar("I",NULL,&I,300,NULL);
	//cvCreateTrackbar("D",NULL,&D,5,NULL);
	//cvCreateTrackbar("kN",NULL,&kN,5,NULL);
}

void droneVision::resetParam(int,void* param){
	//if(reset==1){
	 droneVision* drone=static_cast<droneVision*>(param);
	 drone->kernel_size=1;
//	 reset=0;//reset param
	 drone->Hmin=20,drone->Hmax=66;//HSV threshold
	 drone->Smin=100,drone->Smax=255;
	 drone->Vmin=100,drone->Vmax=255;
	 drone->areaThres=15;
	 drone->epsilon=25;
	 drone->KNN_k=10;
         //drone->P=4;
         //drone->I=150;
        // drone->D=0;
         //drone->kN=2;
	//setTrackbarPos("Reset","Param Control",reset);
	cvSetTrackbarPos("H min",NULL,drone->Hmin);
	cvSetTrackbarPos("H max",NULL,drone->Hmax);
	cvSetTrackbarPos("S min",NULL,drone->Smin);
	cvSetTrackbarPos("S max",NULL,drone->Smax);
	cvSetTrackbarPos("V min",NULL,drone->Vmin);
	cvSetTrackbarPos("V max",NULL,drone->Vmax);
	cvSetTrackbarPos("KernelSize",NULL,drone->kernel_size);
	cvSetTrackbarPos("Area x100",NULL,drone->areaThres);
	cvSetTrackbarPos("Approx eps",NULL,drone->epsilon);
	cvSetTrackbarPos("KNN_k",NULL,drone->KNN_k);
	//cvSetTrackbarPos("P",NULL,drone->P);
	//cvSetTrackbarPos("I",NULL,drone->I);	
	//cvSetTrackbarPos("D",NULL,drone->D);
	//cvSetTrackbarPos("kN",NULL,drone->kN);
	//}
}

void droneVision::learnCallback(int, void* param){
	droneVision* drone=static_cast<droneVision*>(param);
	drone->LEARN=!drone->LEARN;
	//if(drone->DETECT) drone->DETECT=!drone->DETECT;
}
void droneVision::detectCallback(int, void* param){
	droneVision* drone=static_cast<droneVision*>(param);
	drone->DETECT=!drone->DETECT;
	//if(drone->LEARN) drone->LEARN=!drone->LEARN;
}

void droneVision::navdataCallback(const ardrone_autonomy::Navdata& msg){
stringstream ss;
stringstream sss;
string str;
ss<<"State: ";
switch(msg.state){
case 0:
ss<<"   Unknown";
break;
case 1: 
ss<<"    Inited";
break;
case 2:
ss<<"    Landed";
break;
case 3:
ss<<"    Flying";
break;
case 4:
ss<<"  Hovering";
break;
case 5:
ss<<"   Test(?)";
break;
case 6:
ss<<"Taking off";
break;
case 7:
ss<<"    Flying";
break;
case 8:
ss<<"   Landing";
break;
case 9:
ss<<"  Looping?";
break;
}

altitude=msg.altd;
rotX=msg.rotX;
rotY=msg.rotY;
rotZ=msg.rotZ;
ss<<" | Battery: "<<setw(3)<<msg.batteryPercent<<"%"<<" | Altitude: "<<setw(4)<<msg.altd<<" mm | Orientation: "<<setw(4)<<setprecision(3)<<msg.rotZ<<" deg | Linear: "<<"("<<setw(4)<<msg.vx<<","<<setw(4)<<msg.vy<<","<<setw(4)<<msg.vz<<")";
displayStatusBar("MainWindow",ss.str(),100);//display status
}

