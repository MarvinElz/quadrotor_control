#include "ros/ros.h"
#include "PID.h"	

/*
	Aufruf über ähnlich wie:
	pid_.x.init(ros::NodeHandle(controller_nh, "xy"));
*/

PID::PID( const ros::NodeHandle &nh_) : nh(nh_){	
	init( 0, 0 );	
}

PID::PID( const ros::NodeHandle &nh_, float x0, float x1) : nh(nh_){	
	init( x0, x1 );	
}

PID::~PID()
{
	
}

void PID::init( float x0, float x1)
{
	nh.getParam("P", P);
	nh.getParam("I", I);
	nh.getParam("D", D);
	nh.getParam("N", N);
	ROS_INFO("new PID-Controller: %f, %f, %f, %f", P, I, D, N);
	x[0] = x0;
	x[1] = x1;
}

void PID::setInput( double u_ )
{
	u = u_;
}

void PID::updateState( double dt )
{
	double x_dot[2];
	x_dot[0] = u - N * x[0];
	x_dot[1] = u;

	x[0] += x_dot[0] * dt;
	x[1] += x_dot[1] * dt;
}


double PID::getOutput( void )
{
	return P * u + ( N * u - N*N * x[0] ) * D + I * x[1];
}
