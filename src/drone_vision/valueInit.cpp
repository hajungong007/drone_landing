#include "droneVision.hpp"
void droneVision::valueInit(){
	kernel_size=1;
	//int reset=0;//reset param
	Hmin=0;Hmax=84;//HSV threshold
	Smin=0;Smax=255;
	Vmin=100;Vmax=255;
	areaThres=15;//contour area threshold
	epsilon=25;
	RectDetected=false;
	isInside=0;
	LEARN=false;
	DETECT=false;
	trainNumber=0;
	trainCount=0;
	numberSize=32;
	KNN_k=10;
	TNPS=100;
	CNUM=10;
	responses.create(TNPS*CNUM,1,CV_32FC1);
		for(int i=0;i<CNUM;i++){
		responses(Range(i*TNPS,(i+1)*TNPS),Range(0,1))=Scalar(i);
	}
	
	trainData.create(TNPS*CNUM,32*32,CV_32FC1);
	Mat trainData_int=imread("trainData.pbm",0);
	if((trainData_int.rows*trainData.cols)!=0) {
		cout<<"Train data loaded!"<<endl;
		trainData_int.convertTo(trainData,CV_32F);
		bool trained=knn.train(trainData,responses);
		if(trained==false)	cout<<"Fail to train KNN classifier!"<<endl;
	}else 	cout<<"No train data available!"<<endl;
		
	for(int i=0;i<CNUM;i++){
		responses(Range(i*TNPS,(i+1)*TNPS),Range(0,1))=Scalar(i);
	}
	responses.create(TNPS*CNUM,1,CV_32FC1);
	for(int i=0;i<CNUM;i++){
		responses(Range(i*TNPS,(i+1)*TNPS),Range(0,1))=Scalar(i);
	}
	}
