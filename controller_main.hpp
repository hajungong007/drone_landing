#ifndef CONTROLLER_MAIN_HPP
#define CONTROLLER_MAIN_HPP
#include "ros/ros.h"
#include <math.h>
//
//全部采用地磁坐标系，除了speed_geo为机身坐标系;
#include "pid.hpp"
#include "ARDrone.hpp"
#include "getRoute.hpp"
#include "drone_object_ros.h"
#include <ardrone_autonomy/Navdata.h>
#include <iostream>
#include<nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <crazy_landing/error_xy.h>
#include <crazy_landing/num_detected.h>
#include <crazy_landing/pid.h>
#include <crazy_landing/pic_direction.h>
void set_altitude(float,ros::NodeHandle& n);
void set_altitude_quick(float ,ros::NodeHandle& n);

void getnav(const ardrone_autonomy::Navdata& );
void gettarget(const crazy_landing::error_xy& );
void getnum(const crazy_landing::num_detected& );
void getpos(const nav_msgs::Odometry& );
void getpara(const crazy_landing::pid& );

void coord_geo2dro(float& ,float& ,float& , float& ,ros::NodeHandle& n);
void coord_dro2geo(float& ,float& ,float& , float& ,ros::NodeHandle& n);
void coord_indoor2geo(float& ,float& ,float& , float& ,float );
void map_indoor2geo(point *, point *,float );
#endif
