#include "controller_main.hpp"
#define CLIP3(_n1, _n,  _n2) {if (_n<_n1) _n=_n1;  if (_n>_n2) _n=_n2;}
#define LANDSPEED -0.8
using namespace std;
//should be values after pid algorithm


float errorx;//projection on direction x of the local translation vector
float errory;//projection on direction y of the local translation vector
float errorz;//the altitude standard -projection on direction z of the local translation vector
float errangle;//by picture
//float p,i,d,kn;
//the acceptable domains
//float proportion;//proportion=1m 在0.1速度下的秒数
float thresholdstartx=0.035;
float thresholdstarty=0.035;
float thresholdx=0.035;
float thresholdy=0.035;
float thresholdz=0.5;
float threshold=0.1;
//float thresholddistancex=0.2;
float thresholddistance=0.3;
float thresholdangle=float(5)/180.0*3.1415927;
float speed_geo_x,speed_geo_y,speed_dro_x,speed_dro_y,speed_z,speed_angle;
float dist_integrale[2];
PIDController pid_angle,pid_x,pid_y,pid_z,pid_x_move,pid_y_move;//关于水平angle的pid都需要换成pid_angle.getOutput()
//float direction;//[-1,1]direction of forward of the UAV
//float directionsetted;//=float(162.0)/180.0*3.1415927;//地面方框与东南西北夹角
float altitude_setted=2.3;//图像识别定高
float altitude_landing=0.3; //降落最小高度
float timestampnav,timestamptarget,timestampnum,timestamppos,timestampz,timestamppicdirect;
float pos_now[3];
float pos_start[3];
ARDrone drone;
//DroneObjectROS drone_key;
int numdetected=0,numwanted=1;
int num_for_now;
bool is_target_detected,isdetected,isdirection;
bool isconnected;
//point map_geo[10];
point map_indoor[10];
//route_data route[][];//between every two numbers(route[2][4] route[4][2]...)



void getnav(const ardrone_autonomy::Navdata& navPtr) {
//        direction = float(navPtr.rotZ)/180.0*3.1415927;
        errorz = float(navPtr.altd)/1000;
        timestampnav=float(navPtr.tm)/1000000;
}

void gettarget(const crazy_landing::error_xy& errPtr){
    errorx = float(errPtr.err_x)/1000;
    errory = float(errPtr.err_y)/1000;
    is_target_detected=errPtr.is_target_detected;
    timestamptarget = float(errPtr.header.stamp.sec)+floor(errPtr.header.stamp.nsec*100.0)/10000.0;
}
void getnum(const crazy_landing::num_detected & errPtr){
    isdetected=errPtr.is_detected;
    num_for_now=errPtr.num;
}
void getpos(const nav_msgs::Odometry& odoPtr){
    pos_now[0]=odoPtr.pose.pose.position.x;
    pos_now[1]=odoPtr.pose.pose.position.y;
    pos_now[2]=odoPtr.pose.pose.position.z;
    timestamppos=float(odoPtr.header.stamp.sec)+floor(odoPtr.header.stamp.nsec*100.0)/10000.0;
}
void getpicdirection(const crazy_landing::pic_direction& pic_directPtr){
    errangle=pic_directPtr.err_angle;
    isdirection=pic_directPtr.is_direction;
    timestamppicdirect=float(pic_directPtr.header.stamp.sec)+floor(pic_directPtr.header.stamp.nsec*100.0)/10000.0;
}

void PIDcorrect_direc(){
     ros::spinOnce();
        if(isdirection==false) ROS_INFO("error direction is not detected!\n");
        while((abs(errangle)>thresholdangle) && (isdirection==true) && ros::ok())
        {
            while((abs(errangle)>thresholdangle) && (isdirection==true) && ros::ok())
           {
	        //pid_angle.setParam(p,i,d,kn); 
            speed_angle=pid_angle.getOutput(errangle,timestampnav);//dt可能每次数据间隔不一样
            //方向可能有问题
            //pid_z.setParam(p,i,d,kn);
            
	        //speed_z=pid_z.getOutput(-errorz+altitude_setted,timestamppicdirect);
            //CLIP3(-0.5, speed_z,  0.5);			            
            drone.move(0.00,0.00,0.00,speed_angle);//pay attention to the symbol +- of direction
            //do we need a pause after each adjust?
            usleep(500000);
            ros::spinOnce();
            ROS_INFO("pic_direction error is %f\n",errangle);
            ROS_INFO("speed_angle= %f\n",speed_angle);
            }
            drone.hover();
            usleep(500000);
            ros::spinOnce();
        }
    drone.hover();
}
void PIDcorrect_alt(){
     ros::spinOnce();
     while(((abs(errorz-altitude_setted)>thresholdz)) && ros::ok())
        {
            while(((abs(errorz-altitude_setted)>thresholdz)) && ros::ok())
           { 
	        speed_z=pid_z.getOutput(-errorz+altitude_setted,timestampnav);
            CLIP3(-0.5, speed_z,  0.5);			            
            drone.move(0.00,0.00,speed_z,0.0);//pay attention to the symbol +- of direction
            //do we need a pause after each adjust?
            usleep(500000);
           // drone.hover();
            ros::spinOnce();
            ROS_INFO("speed_z= %f\n",speed_z);
            }
            drone.hover();
            usleep(500000);
            ros::spinOnce();
        }
}

void PIDcorrect_pic(float thresx,float thresy){
                ros::spinOnce();//get errorx errory//图像识别？//需要知道时间'
                float timeold=timestamptarget;
                ros::spinOnce();
		        float errorx_old,errory_old;
    while((abs(errorx)>thresx)||(abs(errory)>thresy)||(isdirection==false))
            {
                while((abs(errorx)>thresx)||(abs(errory)>thresy)||(isdirection==false))
                {
	                //pid_x.setParam(p,i,d,kn);
                //pid_y.setParam(p,i,d,kn);  
                    while(timestamptarget==timeold){ 
                        drone.hover();
                        ros::spinOnce();    
			            ROS_INFO("frame lost!\n");                    
                    }
                    timeold=timestamptarget; 
		    if( (errorx*errorx_old<0) || (errory*errory_old<0)) {
				drone.hover();
				ros::spinOnce();			
            }
        
		        speed_dro_x=pid_x.getOutput(errorx,timestamptarget);
                speed_dro_y=pid_y.getOutput(errory,timestamptarget);
                    CLIP3(-0.5, speed_dro_x,  0.5);
                    CLIP3(-0.5, speed_dro_y,  0.5);
                    drone.move(speed_dro_x,speed_dro_y,0.0,0.0);
                    usleep(300000);
                   // drone.hover();
                    ROS_INFO("errorx= %f,errory= %f\n",errorx,errory);
                    ROS_INFO("speed_dro_x= %f,speed_dro_y= %f\n",speed_dro_x,speed_dro_y);
		    errorx_old=errorx;errory_old=errory;
                    ros::spinOnce();
                }
        drone.hover(); 
        usleep(500000);
        ros::spinOnce();
    }
}
void PIDcorrect_pic_landing(){
            ros::spinOnce();
         //while(abs(errorz-altitude_landing)>thresholdz){
                PIDcorrect_pic(thresholdx,thresholdy);    
               // PIDcorrect_direc();
		        drone.land();		
}
void movetarget(int num_from, int num_dest ){
            ROS_INFO("Move to the target %d.\n",num_dest);
            ros::spinOnce();
            ROS_INFO("pos_start=%f %f",pos_now[0],pos_now[1]);
            float dist_integrale_norm;
            routedata route_for_now;
            route_for_now=route(map_indoor,num_from);
            pos_start[0]=pos_now[0];pos_start[1]=pos_now[1];pos_start[2]=pos_now[2];
            ROS_INFO("route_for_now_dist_total: %f\n",route_for_now.dist_total);
            
                speed_dro_x=route_for_now.dist_x/route_for_now.dist_total * 0.1;

                speed_dro_y=route_for_now.dist_y/route_for_now.dist_total * 0.1;
				
                CLIP3(-0.3,speed_dro_x ,  0.3);
                CLIP3(-0.3,speed_dro_y ,  0.3);
                
                drone.move(speed_dro_x,speed_dro_y,0.00,0.00);
                
                usleep(flightT[num_from]*1000000);
                ROS_INFO("route_x= %f, route_y= %f",route_for_now.dist_x,route_for_now.dist_y);
           
                ros::spinOnce();
            */
            while(((route_for_now.dist_total-sqrt((pos_now[0]-pos_start[0])*(pos_now[0]-pos_start[0])+(pos_now[1]-pos_start[1])*(pos_now[1]-pos_start[1])))>(thresholddistance)) && ros::ok()){
				//pid_x.setParam(p,i,d,kn); 
				//pid_y.setParam(p,i,d,kn); 
                speed_dro_x=route_for_now.dist_x/route_for_now.dist_total * 0.1;
                speed_dro_y=route_for_now.dist_y/route_for_now.dist_total * 0.1;
                //coord_geo2dro(speed_geo_x,speed_geo_y,speed_dro_x,speed_dro_y);
                //CLIP3(-0.3,speed_dro_x ,  0.3);
                //CLIP3(-0.3,speed_dro_y ,  0.3);
                drone.move(speed_dro_x,speed_dro_y,0.00,0.00);
                usleep(200000);
                //drone.hover();
                ROS_INFO("speed_geo_x_move= %f,speed_geo_y_move= %f\n",speed_geo_x,speed_geo_y);
                ROS_INFO("route_x= %f, route_y= %f",route_for_now.dist_x,route_for_now.dist_y);
                ROS_INFO("dist_integrale_norm %f\n",sqrt((pos_now[0]-pos_start[0])*(pos_now[0]-pos_start[0])+(pos_now[1]-pos_start[1])*(pos_now[1]-pos_start[1])));
                ros::spinOnce();
                //dist_integrale[0]=pos_now[0]-pos_start[0];
                //dist_integrale[1]=pos_now[1]-pos_start[1];
                ROS_INFO("pos_now=%f %f , pos_start=%f %f",pos_now[0],pos_now[1],pos_start[0],pos_start[1]);
                   ROS_INFO(" total=%f",route_for_now.dist_total);
            }
	drone.hover();
}


int main(int argc,char* argv[]){
    ros::init(argc, argv, "drone_controller");
    ros::NodeHandle n;
    drone.ARDrone_init(n);
   // drone_key.initROSVars(n);
    ros::Subscriber nav_sub = n.subscribe("ardrone/navdata", 1, getnav);//is there already library
    ros::Subscriber target_sub = n.subscribe("error_xy", 1, gettarget);
    ros::Subscriber num_sub = n.subscribe("num_detected", 1, getnum);
    ros::Subscriber pos_sub = n.subscribe("ardrone/odometry",1,getpos);
    ros::Subscriber picdirection_sub = n.subscribe("pic_direction",1,getpicdirection);   
    //ros::Subscriber para_sub=n.subscribe("pid",1,getpara);
    sleep(4);
    //ros::spinOnce();
    //sleep(2);
    //directionsetted=direction;
    pid_angle.setParam(1.0,0.0,0.01,2);//待定
    pid_x.setParam(0.3,0.0,0.2,2);//待定
    pid_y.setParam(0.3,0.0,0.2,2);//待定
    pid_z.setParam(1.0,0.0,0.0,2);//待定
    pid_x_move.setParam(0.5,0.0,0.0,2);
    pid_y_move.setParam(0.5,0.0,0.0,2);
    set_route_data(map_indoor);
    //map_indoor2geo(map_indoor ,map_geo ,-directionsetted);//转换map至地磁坐标
     for(numdetected=0;numdetected<9;numdetected++){
        drone.takeOff();//takeoff and keep on a certain altitude
        ROS_INFO("Taking off done.\n");
        sleep(5);
        //ros::spinOnce();
        //directionsetted=direction;
        //ROS_INFO("direction error is %f,direction is %f",directionsetted-direction,direction);
        PIDcorrect_alt();
//correct takeoff deviation
       drone.move(-0.1,0.0,0.00,0.00);
        usleep(1000000);
        ROS_INFO("Corrections of the  altitude done.\n");
	    drone.hover();
        //sleep(1);
        //ros::spinOnce();
        //起飞后对起点进行一次修正；
        PIDcorrect_pic(thresholdstartx,thresholdstarty);
        ROS_INFO("Correcting by the picture done.\n");
	               //correct again the  alt
	        //PIDcorrect_alt();
    	  //  PIDcorrect_direc();
            //sleep(1);
	   ROS_INFO("Corrections of the direction and altitude done.\n");
	   //drone.hover();
	   //sleep(1);
       movetarget(numdetected,numdetected+1);
       //sleep(1);
       //numdetected++;
       ROS_INFO("Moving done.\n");
       drone.hover();
            ROS_INFO("Start to landing\n");
           // drone.hover();
            PIDcorrect_pic_landing();
            //ROS_INFO("Landing...\n");
            //drone.land();
            //sleep(3);
            //ROS_INFO("Landed.\n");
}	
   // }
   // }
   return 0;
}

