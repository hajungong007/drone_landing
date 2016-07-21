#ifndef ARDRONE_ROS_H
#define ARDRONE_ROS_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

/**
 * @brief A simple class to send the commands to the drone through 
 * the corresponding topics
 */

class DroneObjectROS{
protected:
    
public:
    DroneObjectROS(){}
    //DroneObjectROS(ros::NodeHandle& node){
     //   initROSVars(node);
    //}

    bool isFlying;
    bool isPosctrl;
    bool isVelMode;
    
    ros::Publisher pubTakeOff;
    ros::Publisher pubLand;
    ros::Publisher pubReset;
    ros::Publisher pubCmd;
   ros::Publisher pubPosCtrl;
    ros::Publisher pubVelMode;
    
    geometry_msgs::Twist twist_msg;
    
    void initROSVars(ros::NodeHandle& node);
    bool reset();
    bool takeOff();
    bool land();
    bool hover();
    bool posCtrl(bool on);
    bool velMode(bool on);

    bool move(float v_lr, float v_fb, float v_du, float w_lr);
    bool moveTo(float x, float y, float z);
    bool pitch(float speed );//0.2
    bool roll(float speed );//0.2
    bool rise(float speed);//0.1
    bool yaw(float speed);//0.1
};

#endif // ARDRONE_ROS_H
