#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "msg/Pose.h"
#include "sensor_msgs/Joy.h"

enum { X, Y, Z, YAW };

struct vel
{
	union linear{
		double V[3];
		struct{double x, y, z;};
	}linear;
	union angular{
		double V[3];
		struct{double x, y, z;};
		struct{double phi, theta, psi;};
		struct{double roll, pitch, yaw;};
	}angular;
} typedef vel;

struct pose
{	
	union position{
		double V[3];
		struct{double x, y ,z;};
	} position;
	union orientation{
		double V[3];
		struct{double x, y, z;};
		struct{double phi, theta, psi;};	
		struct{double roll, pitch, yaw;};	
	}
	
} typedef pose;

double U[4];
double vel_desired[4];
double vel_max[4];
vel vel_measure = {{0, 0, 0}, {0, 0, 0}}
pose pose_measure = {{0, 0, 0}, {0, 0, 0}};

using namespace PID;

void berechne_UAV_sollgeschwindigkeit(const sensor_msgs::Joy::ConstPtr& msg)
{
	vel_desired[X]   =   msg.axes[4] * vel_max[X];
	vel_desired[Y]   = - msg.axes[3] * vel_max[Y];
	vel_desired[Z] 	 =   msg.axes[2] * vel_max[Z];
	vel_desired[YAW] = - msg.axes[0] * vel_max[YAW];
}

void callback_vel_measure( const geometry_msgs::Twist::ConstPtr& msg )
{
	vel_measure.linear.x  = msg.linear.x;
	vel_measure.linear.y  = msg.linear.y;
	vel_measure.linear.z  = msg.linear.z;

	vel_measure.angular.x = msg.angular.x;
	vel_measure.angular.y = msg.angular.y;
	vel_measure.angular.z = msg.angular.z;
}

void callback_pose_measure( const msg::Pose::ConstPtr& msg )
{
	pose_measure.position.x = msg.position.x;
	pose_measure.position.y = msg.position.y;
	pose_measure.position.z = msg.position.z;

	pose_measure.orientation.phi 	= msg.orientation.x;
	pose_measure.orientation.theta 	= msg.orientation.y;
	pose_measure.orientation.psi 	= msg.orientation.z;
}

void callback_rc_signal_delayed( const sensor_msgs::Joy::ConstPtr& msg )
{
	berechne_UAV_sollgeschwindigkeit( msg );
}

// Servie Aufruf zum Starten der Berechnung
bool propagate()
{
	// TODO: dt berechnen
	double dt;

	pid_vz.setInput( vel_desired[Z] - vel_measure.linear.z );
	pid_vz.updateState( dt );
	U[0] = pid_vz.getOutput();

	pid_vyaw.setInput( vel_desired[YAW] - vel_measure.angular.yaw );
	pid_vyaw.updateState( dt );
	U[3] = pid_vyaw.getOutput();

	
	// TODO: Koordinatentransformation Body -> Init
	// TODO: pid_vx und pid_vy berechnen
	// TODO: Bewegungsgleichung berechnen
	// TODO: Saturation
	// TODO: pid_phi und pid_theta berechnen

	// TODO: Publish Steuergrößen
	return true;
}



int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Controller");

	ros::NodeHandle nh("Controller");

	// Parameter für Max-Geschwindigkeit holen
	nh.getParam("vx_max"	, &vel_max[X]);
	nh.getParam("vy_max"	, &vel_max[Y]);
	nh.getParam("vz_max"	, &vel_max[Z]);
	nh.getParam("vyaw_max"	, &vel_max[YAW]);

	PID pid_vx 		= new PID( ros::NodeHandle(nh, "vxy"	) );
	PID pid_vy 		= new PID( ros::NodeHandle(nh, "vxy"	) );
	PID pid_vz 		= new PID( ros::NodeHandle(nh, "vz"		) );
	PID pid_vyaw 	= new PID( ros::NodeHandle(nh, "vyaw"	) );
	PID pid_roll 	= new PID( ros::NodeHandle(nh, "roll"	) );
	PID pid_pitch 	= new PID( ros::NodeHandle(nh, "pitch"	) );

	ros::Subscriber sub = nh.subscribe("rc_signal_delayed", 10, callback_rc_signal_delayed);
	ros::Subscriber sub = nh.subscribe("vel_measure", 10, callback_vel_measure);
	ros::Subscriber sub = nh.subscribe("pose_measure", 10, callback_pose_measure);

	ros::ServiceServer service = n.advertiseService("controller_prop", propagate);

	ros::spin();

	return 0;
}