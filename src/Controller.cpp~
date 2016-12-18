#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "quadrotor_control/pose.h"
#include "sensor_msgs/Joy.h"
#include "PID.h"
#include "quadrotor_control/manipulated_variables.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#include "Controller.h"

double ABS( double d ){
	if( d < 0 ) return (-1.0)*d;
	else return d;
}

void berechne_UAV_sollgeschwindigkeit(const sensor_msgs::Joy::ConstPtr& msg)
{	
	//ROS_INFO( "INPUT: %f, %f, %f, %f", msg->axes[4], msg->axes[3], msg->axes[1], msg->axes[0] );
	if( ABS(msg->axes[4]) < 0.2 )
		vel_desired[X] = 0.0;
	else
		vel_desired[X]   =   msg->axes[4] * vel_max[X];

	if( ABS(msg->axes[3]) < 0.2 )
		vel_desired[Y] = 0.0;
	else
		vel_desired[Y]   = - msg->axes[3] * vel_max[Y];

	if( ABS(msg->axes[1]) < 0.2 )
		vel_desired[Z] = 0.0;
	else
		vel_desired[Z] 	 =   msg->axes[1] * vel_max[Z];

	if( ABS(msg->axes[0]) < 0.2 )
		vel_desired[YAW] = 0.0;
	else
		vel_desired[YAW] = - msg->axes[0] * vel_max[YAW];
	//ROS_INFO( "SOLL_NEU: %f, %f, %f, %f", vel_desired[X], vel_desired[Y], vel_desired[Z], vel_desired[YAW]);
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

/*
	Parameter: psi		 .. Aktueller Gierwinkel Psi
		   vel_desired_B .. Gewünschte Geschwindigkeit im B-System
		   vel_desired_D .. Resultierende Geschwindigkeit im N-System
*/
void transform_coord( const double psi, const double *vel_desired_B, double *vel_desired_N )
{
	double sin_psi = sin( psi );
	double cos_psi = cos( psi );

	vel_desired_N[X]   = vel_desired_B[X] * cos_psi - vel_desired_B[Y] * sin_psi;
	vel_desired_N[Y]   = vel_desired_B[X] * sin_psi + vel_desired_B[Y] * cos_psi;	 
}

// necessary tilt for desired acceleration
void motion_equation( const double psi, const double ax, const double ay, double& roll_desired, double& pitch_desired )
{
	double sin_psi = sin( psi );
	double cos_psi = cos( psi );

	double m = 0.65; // TODO: Ueber PARAMETER einlesen, oder??
	double f;
	if(U[0] != 0.0)
	   f = m/U[0];
        else
	   f = m/5;
	
	roll_desired   = f*( -ax * sin_psi + ay * cos_psi );
	pitch_desired  = f*( -ax * cos_psi - ay * sin_psi );
}

// Servie Aufruf zum Starten der Berechnung
bool propagate( std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp )
{
	// Simulation wurde gerade gestartet
	if( req.data )	
		last_Prop = ros::Time::now();

	//ROS_INFO( "Start Propagation" );	
	ros::Time now = ros::Time::now();
   	double dt = (now - last_Prop).toSec();   
   	last_Prop = now;	

	// Sollbeschleunigungen	
	pid_vz->setInput( vel_measure.linear.z - vel_desired[Z] );	
	U[0] = pid_vz->getOutput();
	pid_vz->updateState( dt );

	pid_vyaw->setInput( vel_desired[YAW] - vel_measure.angular.yaw );	
	U[3] = pid_vyaw->getOutput();
	pid_vyaw->updateState( dt );
	
	// Koordinatentransformation B -> N
	double vel_desired_N[2];
	transform_coord( pose_measure.orientation.psi, vel_desired, vel_desired_N );
	
	double ax, ay;

	pid_vx->setInput( vel_desired_N[X] - vel_measure.linear.x );	
	ax = pid_vx->getOutput();
	pid_vx->updateState( dt );

	pid_vy->setInput( vel_desired_N[Y] - vel_measure.linear.y );
	ay = pid_vy->getOutput();
	pid_vy->updateState( dt );

	//ROS_INFO("AX: %f, AY: %f", ax, ay);

	//ROS_INFO( "SOLL: %f, %f, %f, %f", vel_desired[X], vel_desired[Y], vel_desired[Z], vel_desired[YAW]);
    
	// Bewegungsgleichung berechnen
	double roll_desired = 0.0;
	double pitch_desired = 0.0;
	motion_equation( pose_measure.orientation.psi, ax, ay, roll_desired, pitch_desired );

	// Saturation
        if( roll_desired >  20.0/180.0*3.1415) roll_desired  = 20.0/180.0*3.1415;
	if( roll_desired <- 20.0/180.0*3.1415) roll_desired  =-20.0/180.0*3.1415;
        if( pitch_desired > 20.0/180.0*3.1415) pitch_desired = 20.0/180.0*3.1415;
	if( pitch_desired <-20.0/180.0*3.1415) pitch_desired =-20.0/180.0*3.1415;

	pid_roll->setInput( roll_desired - pose_measure.orientation.roll );
	U[1] = pid_roll->getOutput();
	pid_roll->updateState( dt );

	pid_pitch->setInput( pitch_desired - pose_measure.orientation.pitch );
	U[2] = pid_pitch->getOutput();
	pid_pitch->updateState( dt );

	// Publish Steuergrößen
	quadrotor_control::manipulated_variables U_out;
	U_out.U.push_back(U[0]);
	U_out.U.push_back(U[1]);
	U_out.U.push_back(U[2]);
	U_out.U.push_back(U[3]);
	pub.publish(U_out);

	//ROS_INFO( "END PROPAGATION" );
        //ROS_INFO( "U1: %f, U2: %f, U3: %f, U4: %f", U[0], U[1], U[2], U[3] );
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
        last_Prop = ros::Time::now();

	pid_vx 	= new PID( ros::NodeHandle(nh, "vxy"	) );
	pid_vy 	= new PID( ros::NodeHandle(nh, "vxy"	) );
	pid_vz 	= new PID( ros::NodeHandle(nh, "vz"	), 3, 3 );
	pid_vyaw 	= new PID( ros::NodeHandle(nh, "vyaw"	) );
	pid_roll 	= new PID( ros::NodeHandle(nh, "roll"	) );
	pid_pitch 	= new PID( ros::NodeHandle(nh, "pitch"	) );

	ros::Subscriber sub1 = nh.subscribe("/rc_signal_delayed", 10, callback_rc_signal_delayed);
	ros::Subscriber sub2 = nh.subscribe("/vel_measure", 10, callback_vel_measure);
	ros::Subscriber sub3 = nh.subscribe("/pose_measure", 10, callback_pose_measure);

	ros::ServiceServer service = nh.advertiseService("controller_prop", propagate);

	pub = nh.advertise<quadrotor_control::manipulated_variables>("/stellgroessen", 100);

	ROS_INFO( "Controller Init done" );

	ros::spin();

	return 0;
}
