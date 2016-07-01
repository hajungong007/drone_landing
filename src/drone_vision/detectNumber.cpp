#include "droneVision.hpp"
void droneVision::detectNumber(){
	/*Collect sample*/
	if(LEARN==1){
		/*Transform ROI to ROW SAMPLE form and add to sample*/
		int count=0;
		MatIterator_<uchar> it, end;
            for( it = ROI.begin<uchar>(), end = ROI.end<uchar>(); it != end; ++it)
					{
						trainData.at<float>(trainNumber*TNPS+trainCount,count)=*it;
						count++;
			    		}
		trainCount++;
		
	if(trainCount==TNPS){	
		if(trainNumber==(CNUM-1)){
			imwrite("trainData.pbm",trainData);
			LEARN=0;
			cout<<"Learning finished!"<<endl;
			bool trained=knn.train(trainData,responses);
			if(trained==false){
				cout<<"Fail to train KNN classifier!"<<endl;
			}

		}
		else{
			cout<<trainNumber<<" finish learning! Next number: "<<trainNumber+1<<endl;
			LEARN=0;
			trainNumber++;
			trainCount=0;
		}
	}
	
	return;
	}
	
	/*Detect number*/
	if(DETECT==1){
	/*Do the KNN and publish resuld*/
	Mat samples(1,numberSize*numberSize,CV_32FC1);
	int count=0;
	MatIterator_<uchar> it, end;
         for( it = ROI.begin<uchar>(), end = ROI.end<uchar>(); it != end; ++it)
				{
					samples.at<float>(count)=*it;
					count++;
			    	}
	detectedNumber=knn.find_nearest(samples,KNN_k);
	cout<<detectedNumber<<endl;
	crazy_landing::num_detected nummsg;
	nummsg.header.stamp=ros::Time::now();
	nummsg.num=detectedNumber;
	nummsg.is_detected=true;
	num_pub.publish(nummsg);
	}
	

}
