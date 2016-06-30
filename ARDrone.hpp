#ifndef ARDRONE_HPP
#define ARDRONE_HPP
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


using namespace std;

class ARDrone {

public:
	ARDrone(void ){}
	~ARDrone(void){}
	
	bool isFlying;

	bool takeOff();
	bool land();
	bool hover();
	bool reset();

    ////////////////////////////////////////////////////////////SZ
        geometry_msgs::Twist twist_msg;
        geometry_msgs::Twist twist_msg_hover;
     //   geometry_msgs::Twist twist_msg_pshover;
        std_msgs::Empty emp_msg;



                       ros::Publisher pub_empty_land;
                       ros::Publisher pub_twist;
                       ros::Publisher pub_empty_takeoff;
                       ros::Publisher pub_empty_reset;
        ///////////////////////////////////////////////////////////////

	// move command
	// v_lr = velocity left			(-1) to right		(+1)
	// v_fb = velocity forwards		(-1) to backwards	(+1)
	// v_du = velocity downwards	(-1) to upwards		(+1)
	// w_lr = angular velocity left (-1) to right		(+1)
	void ARDrone_init(ros::NodeHandle &);
	bool move(float v_lr, float v_fb, float v_du, float w_lr);
	bool moveLeft(float speed = -0.2);
	bool moveRight(float speed = 0.2);
	bool moveForward(float speed = 0.1);
	bool moveBackward(float speed = 0.1);
	bool moveUp(float speed = 0.1);
	bool moveDown(float speed = 0.1);
	bool turnLeft(float speed = 0.1);
	bool turnRight(float speed = 0.1);
};
#endif
