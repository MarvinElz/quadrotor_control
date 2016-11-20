#include "ros/ros.h"
// #include rc_signal_msg

using namespace PID;

void callback_vel_measure()
{
	// TODO: Werte einlesen und speichern
}

void callback_pose_measure()
{
	// TODO: Werte einlesen und speichern
}

void callback_rc_signal(/* rc_signal_msg */)
{
	// TODO: Werte einlesen und speichern
}

double rc_signal[4];

int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Controller");

	ros::NodeHandle nh("Controller");

	PID pid_vx 		= new PID( ros::NodeHandle(nh, "vxy"	) );
	PID pid_vy 		= new PID( ros::NodeHandle(nh, "vxy"	) );
	PID pid_vz 		= new PID( ros::NodeHandle(nh, "vz"		) );
	PID pid_vpsi 	= new PID( ros::NodeHandle(nh, "vpsi"	) );
	PID pid_phi 	= new PID( ros::NodeHandle(nh, "phi"	) );
	PID pid_theta 	= new PID( ros::NodeHandle(nh, "theta"	) );

	ros::Subscriber sub = nh.subscribe("rc_signal_msg", 10, callback_rc_signal);

	while (ros::ok())
	{
		// TODO: pid_vz berechnen
		// TODO: pid_vpsi berechnen
		// TODO: Koordinatentransformation Body -> Init
		// TODO: pid_vx und pid_vy berechnen
		// TODO: Bewegungsgleichung berechnen
		// TODO: Saturation
		// TODO: pid_phi und pid_theta berechnen

		// TODO: Publish Steuergrößen

	}

	return 0;
}