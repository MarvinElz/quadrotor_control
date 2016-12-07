#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "quadrotor_control/pose.h"
#include "sensor_msgs/Joy.h"
#include "PID.h"
#include "quadrotor_control/manipulated_variables.h"
#include "std_srvs/Empty.h"

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
	} orientation;
	
} typedef pose;

double U[4] = {3.0, 0.0, 0.0, 0.0}; // U[0] != 0
double vel_desired[4] = {0.0, 0.0, 0.0, 0.0};
double vel_max[4] = {0.0, 0.0, 0.0, 0.0};
double roll_desired = 0.0;
double pitch_desired = 0.0;
vel vel_measure = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
pose pose_measure = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
double ax, ay;

PID* pid_vx;
PID* pid_vy;
PID* pid_vz;
PID* pid_vyaw;
PID* pid_roll;
PID* pid_pitch;

ros::Publisher pub;


//using namespace PID;

void berechne_UAV_sollgeschwindigkeit(const sensor_msgs::Joy::ConstPtr& msg)
{
	vel_desired[X]   =   msg->axes[4] * vel_max[X];
	vel_desired[Y]   = - msg->axes[3] * vel_max[Y];
	vel_desired[Z] 	 =   msg->axes[2] * vel_max[Z];
	vel_desired[YAW] = - msg->axes[0] * vel_max[YAW];
	ROS_INFO( "SOLL: %f, %f, %f, %f", vel_desired[X], vel_desired[Y], vel_desired[Z], vel_desired[YAW]);
}

void callback_vel_measure( const geometry_msgs::Twist::ConstPtr& msg )
{
	vel_measure.linear.x  = msg->linear.x;
	vel_measure.linear.y  = msg->linear.y;
	vel_measure.linear.z  = msg->linear.z;

	vel_measure.angular.x = msg->angular.x;
	vel_measure.angular.y = msg->angular.y;
	vel_measure.angular.z = msg->angular.z;
}

void callback_pose_measure( const quadrotor_control::pose::ConstPtr& msg )
{
	pose_measure.position.x = msg->position.x;
	pose_measure.position.y = msg->position.y;
	pose_measure.position.z = msg->position.z;

	pose_measure.orientation.phi 	= msg->orientation.x;
	pose_measure.orientation.theta 	= msg->orientation.y;
	pose_measure.orientation.psi 	= msg->orientation.z;
}

void callback_rc_signal_delayed( const sensor_msgs::Joy::ConstPtr& msg )
{
	berechne_UAV_sollgeschwindigkeit( msg );
}

void transform_coord()
{
	double sin_psi = sin(pose_measure.orientation.phi);
	double cos_psi = cos(pose_measure.orientation.phi);

	double temp    = vel_desired[X] * cos_psi - vel_desired[Y] * sin_psi;
	vel_desired[Y] = vel_desired[X] * sin_psi + vel_desired[Y] * cos_psi;
	vel_desired[X] = temp;	 
}

// necessary tilt for desired acceleration
void motion_equation()
{
	double sin_psi = sin(pose_measure.orientation.phi);
	double cos_psi = cos(pose_measure.orientation.phi);

	double m = 0.7; // TODO: Ueber PARAMETER einlesen, oder??
	double f = m/U[0];
	roll_desired   = f*( -ax * sin_psi + ay * cos_psi );
	pitch_desired  = f*( -ax * cos_psi - ay * sin_psi );
}

// Servie Aufruf zum Starten der Berechnung
bool propagate( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
	ROS_INFO( "Start Propagation" );	
	// TODO: dt berechnen
	double dt = 0.005;
	// Sollbeschleunigungen	

	pid_vz->setInput( vel_desired[Z] - vel_measure.linear.z );
	pid_vz->updateState( dt );
	U[0] = pid_vz->getOutput();

	pid_vyaw->setInput( vel_desired[YAW] - vel_measure.angular.yaw );
	pid_vyaw->updateState( dt );
	U[3] = pid_vyaw->getOutput();
	
	// TODO: Koordinatentransformation Body -> Init
	transform_coord();

	pid_vx->setInput( vel_desired[X] - vel_measure.linear.x );
	pid_vx->updateState( dt );
	ax = pid_vx->getOutput();

	pid_vy->setInput( vel_desired[Y] - vel_measure.linear.y );
	pid_vy->updateState( dt );
	ay = pid_vy->getOutput();

	// TODO: Bewegungsgleichung berechnen
	motion_equation();
	
	// TODO: Saturation

	pid_roll->setInput( roll_desired - pose_measure.orientation.roll );
	pid_roll->updateState( dt );
	U[1] = pid_roll->getOutput();

	pid_pitch->setInput( pitch_desired - pose_measure.orientation.pitch );
	pid_pitch->updateState( dt );
	U[2] = pid_pitch->getOutput();

	// TODO: Publish Steuergrößen
	quadrotor_control::manipulated_variables U_out;
	U_out.U.push_back(U[0]);
	U_out.U.push_back(U[1]);
	U_out.U.push_back(U[2]);
	U_out.U.push_back(U[3]);
	pub.publish(U_out);

	ROS_INFO( "END PROPAGATION" );
	return true;
}



int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Controller");

	ros::NodeHandle nh("Controller");

	// Parameter für Max-Geschwindigkeit holen
	nh.getParam("vx_max"	, vel_max[X]);
	nh.getParam("vy_max"	, vel_max[Y]);
	nh.getParam("vz_max"	, vel_max[Z]);
	nh.getParam("vyaw_max"	, vel_max[YAW]);

	pid_vx 	= new PID( ros::NodeHandle(nh, "vxy"	) );
	pid_vy 	= new PID( ros::NodeHandle(nh, "vxy"	) );
	pid_vz 	= new PID( ros::NodeHandle(nh, "vz"	) );
	pid_vyaw 	= new PID( ros::NodeHandle(nh, "vyaw"	) );
	pid_roll 	= new PID( ros::NodeHandle(nh, "roll"	) );
	pid_pitch 	= new PID( ros::NodeHandle(nh, "pitch"	) );

	ros::Subscriber sub1 = nh.subscribe("/rc_signal_delayed", 10, callback_rc_signal_delayed);
	ros::Subscriber sub2 = nh.subscribe("/vel_measure", 10, callback_vel_measure);
	ros::Subscriber sub3 = nh.subscribe("/pose_measure", 10, callback_pose_measure);

	ros::ServiceServer service = nh.advertiseService("controller_prop", propagate);

	pub = nh.advertise<quadrotor_control::manipulated_variables>("stellgroessen", 1000);

	ROS_INFO( "Controller Init done" );

	ros::spin();

	return 0;
}
