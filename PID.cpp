#include "ros/ros.h"
#include "PID.hpp"	

/*
	Aufruf über ähnlich wie:
	pid_.x.init(ros::NodeHandle(controller_nh, "xy"));
*/

PID::PID( const ros::NodeHandle &nh_ ) : nh(nh_){
	last_update = ros::Time::now();
	ROS_INFO("new Controller");
}

PID::~PID()
{
	
}

PID::init()
{
	nh.getParam("P", &P);
	nh.getParam("I", &I);
	nh.getParam("D", &D);
	nh.getParam("N", &N);
	x[0] = 0;
	x[1] = 0;
}

PID::setInput( double u_ )
{
	u = u_;
}

PID::updateState( double dt )
{
	double x_dot[2];
	x_dot[0] = u - N * x[0];
	x_dot[1] = u;

	x[0] += x_dot[0] * dt;
	x[1] += x_dot[1] * dt;

	last_update = time_now;
}


double PID::getOutput( void )
{
	return P * u + ( N * u - N*N * x[0] ) * D + I * x[1];
}