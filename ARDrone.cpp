#include "ARDrone.hpp"
using namespace std;

void ARDrone::ARDrone_init(ros::NodeHandle& n){
	 isFlying=false;
	 pub_empty_land = n.advertise<std_msgs::Empty>("/ardrone/land",1024) ;
     pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel",1024) ;
     pub_empty_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff",1024) ;
     pub_empty_reset = n.advertise<std_msgs::Empty>("/ardrone/reset",1024) ;
 	 pub_empty_takeoff.publish(std_msgs::Empty()); //launches the drone
	 twist_msg_hover.linear.x=0.0;
	 twist_msg_hover.linear.y=0.0;
	 twist_msg_hover.linear.z=0.0;
	 twist_msg_hover.angular.x=0.0;
	 twist_msg_hover.angular.y=0.0;
	 twist_msg_hover.angular.z=0.0;
     pub_twist.publish(twist_msg_hover); 
     pub_empty_land.publish(emp_msg);
     pub_twist.publish(twist_msg_hover);
     pub_empty_reset.publish(emp_msg);
     sleep(2);
     //pub_empty_takeoff.publish(std_msgs::Empty());

}

bool ARDrone::takeOff() {
	if (isFlying)
		return false;

    bool result = true;
    //result = result && ARDrone::sendTrim();
    //result = result && ARDrone::sendTakeOff();

    pub_empty_takeoff.publish(std_msgs::Empty()); //launches the drone
    pub_twist.publish(twist_msg_hover); //drone is flat
    ROS_INFO("Taking off");
    ros::spinOnce();
    isFlying = true;

	//test
    //cout << "take off!" << endl;
	return result;
}

bool ARDrone::land() {
	if (!isFlying)
		return false;

    //return ARDrone::sendLand();
    pub_twist.publish(twist_msg_hover); //drone is flat
    pub_empty_land.publish(emp_msg); //lands the drone
                            ROS_INFO("Landing");
                            ros::spinOnce();
                            sleep(1);
			    isFlying = false;
                            return true;
}
bool ARDrone::reset() {
	if (!isFlying)
		return false;

    //return ARDrone::sendLand();
    pub_twist.publish(twist_msg_hover); //drone is flat
    pub_empty_reset.publish(emp_msg); //lands the drone
                            ROS_INFO("Resting");
                            ros::spinOnce();
                            sleep(1);
			    isFlying = false;
                            return true;
}
bool ARDrone::hover() {
	if (!isFlying)
		return false;

    //return ARDrone::sendHover();
     pub_twist.publish(twist_msg_hover); //drone is flat
                            ROS_INFO("Hovering");
                            ros::spinOnce();
                            sleep(1);
                            return true;
}

bool ARDrone::move(float fb, float lr, float ud, float w) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMove(lr, fb, ud, w);
                            twist_msg.linear.x=fb;
                            twist_msg.linear.y=lr;
                            twist_msg.linear.z=ud;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z= w;
                            pub_twist.publish(twist_msg);
                            //sleep(1);
                            //ARDrone::hover();
                            ROS_INFO("Moving");
    return true;
}

bool ARDrone::moveLeft(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMoveLeft(speed);
    twist_msg.linear.x=0.0;
                            twist_msg.linear.y=speed;
                            twist_msg.linear.z=0.0;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=0.0;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
}

bool ARDrone::moveRight(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMoveRight(speed);
    twist_msg.linear.x=0.0;
                            twist_msg.linear.y=   - speed;
                            twist_msg.linear.z=0.0;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=0.0;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
}

bool ARDrone::moveForward(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMoveForward(speed);
    twist_msg.linear.x= speed;
                            twist_msg.linear.y=0.0;
                            twist_msg.linear.z=0.0;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=0.0;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
}

bool ARDrone::moveBackward(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMoveBackward(speed);
    twist_msg.linear.x= - speed;
                            twist_msg.linear.y=0.0;
                            twist_msg.linear.z=0.0;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=0.0;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
}

bool ARDrone::moveUp(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMoveUp(speed);
    twist_msg.linear.x= 0.0;
                            twist_msg.linear.y=0.0;
                            twist_msg.linear.z= speed;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=0.0;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
}

bool ARDrone::moveDown(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendMoveDown(speed);
    twist_msg.linear.x= 0.0;
                            twist_msg.linear.y=0.0;
                            twist_msg.linear.z= - speed;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=0.0;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
}

bool ARDrone::turnLeft(float speed) {
	if (!isFlying)
		return false;

	twist_msg.linear.x= 0.0;
                            twist_msg.linear.y=0.0;
                            twist_msg.linear.z= 0.0;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z=  speed;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
    //return ARDrone::sendTurnLeft(speed);
    return true;
}

bool ARDrone::turnRight(float speed) {
	if (!isFlying)
		return false;

    //return ARDrone::sendTurnRight(speed);
	twist_msg.linear.x= 0.0;
                            twist_msg.linear.y=0.0;
                            twist_msg.linear.z= 0.0;
                            twist_msg.angular.x=0.0;
                            twist_msg.angular.y=0.0;
                            twist_msg.angular.z= - speed;
                            pub_twist.publish(twist_msg);
                            ROS_INFO("Moving");
                            return true;
    return true;
}
