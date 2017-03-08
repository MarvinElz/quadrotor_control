#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"		// wird hier als 4-dim Array float64 missbraucht
#include "quadrotor_control/manipulated_variables.h"
#include "quadrotor_control/pose.h"
#include "quadrotor_control/kinematics.h"
#include "sensor_msgs/Joy.h"
#include "PID.h"

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#include "Controller.h"

// Masse des UAV (wird benötigt für Regelung)
// siehe Methode 'motion_equation'
double m = 0.65;


double ABS( double d ){
	if( d < 0.0 ) return (-1.0)*d;
	else return d;
}

void berechne_UAV_sollgeschwindigkeit(const sensor_msgs::Joy::ConstPtr& msg)
{		
	if( ABS(msg->axes[X_Axis]) < 0.2 )
		vel_desired[X] = 0.0;
	else
		vel_desired[X]   =   msg->axes[X_Axis] * vel_max[X];

	if( ABS(msg->axes[Y_Axis]) < 0.2 )
		vel_desired[Y] = 0.0;
	else
		vel_desired[Y]   = - msg->axes[Y_Axis] * vel_max[Y];

	if( ABS(msg->axes[Z_Axis]) < 0.2 )
		vel_desired[Z] = 0.0;
	else
		vel_desired[Z] 	 = - msg->axes[Z_Axis] * vel_max[Z];

	if( ABS(msg->axes[Yaw_Axis]) < 0.2 )
		vel_desired[YAW] = 0.0;
	else
		vel_desired[YAW] = - msg->axes[Yaw_Axis] * vel_max[YAW];
/*	
	ROS_INFO("V SOLL: VX: %f, VY: %f, VZ: %f, VPsi: %f", 
		vel_desired[X], 
		vel_desired[Y], 
		vel_desired[Z], 
		vel_desired[YAW]);
*/

// hier publish
	geometry_msgs::Quaternion VSoll;
	VSoll.x = vel_desired[X];
	VSoll.y = vel_desired[Y];
	VSoll.z = vel_desired[Z];
	VSoll.w = vel_desired[YAW];
	pubSoll.publish(VSoll);

}

/*
	Aufruf bei Eintreffen der neuen Pose- und Geschwindigkeitsdaten 
	vom youBot- bzw. Dynamics
	lokale Speicherung der Daten
*/
void callback_kin_measure( const quadrotor_control::kinematics::ConstPtr& msg )
{
	vel_measure.linear.x  = msg->vel.linear.x;
	vel_measure.linear.y  = msg->vel.linear.y;
	vel_measure.linear.z  = msg->vel.linear.z;

	vel_measure.angular.x = msg->vel.angular.x;
	vel_measure.angular.y = msg->vel.angular.y;
	vel_measure.angular.z = msg->vel.angular.z;

	pose_measure.position.x = msg->pose.position.x;
	pose_measure.position.y = msg->pose.position.y;
	pose_measure.position.z = msg->pose.position.z;

	pose_measure.orientation.phi 	= msg->pose.orientation.x;
	pose_measure.orientation.theta 	= msg->pose.orientation.y;
	pose_measure.orientation.psi 	= msg->pose.orientation.z;

}

/*
	Aufruf bei Eintreffen der verzögerten Fernbedienungssignale
*/
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

/*
	Berechnet die für die Geschwindigkeitsvorgaben benötigte Neigung
	in X- und Y-Richtung
*/
void motion_equation( const double psi, const double ax, const double ay, double& roll_desired, double& pitch_desired )
{
	double sin_psi = sin( psi );
	double cos_psi = cos( psi );

	double f;
	// Falls Division durch 0 bevorsteht:
	// f duch einen konstanten Wert ersetzen
	if(U[0] != 0.0)
	   f = m/U[0];
        else
	   f = m/5;
	
	roll_desired   = f*( -ax * sin_psi + ay * cos_psi );
	pitch_desired  = f*( -ax * cos_psi - ay * sin_psi );
}

/*
	Bereitgestellte Servicefunktion:
		berechnet Stellgrößen aus 
			- vel_desired
			- vel_measure, pose_measure
*/
bool propagate( std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp )
{	
// Simulation wurde gerade gestartet
	if( req.data )	
		last_Prop = ros::Time::now();

	ros::Time now = ros::Time::now();
  double dt = (now - last_Prop).toSec(); 
	ROS_INFO("dt: %f", dt);
	if(dt > 0.02) dt = 0.02; 
 
	//dt = 0.005;						// konstante Schrittweite 
 	last_Prop = now;	

	// VZ- und VYaw-Regelung
	pid_vz->setInput( vel_measure.linear.z - vel_desired[Z] );	
	U[0] = pid_vz->getOutput();
	pid_vz->updateState( dt );

	pid_vyaw->setInput( vel_desired[YAW] - vel_measure.angular.yaw );	
	U[3] = pid_vyaw->getOutput();
	pid_vyaw->updateState( dt );
	
	// Koordinatentransformation B -> N
	double vel_desired_N[2];
	transform_coord( pose_measure.orientation.psi, vel_desired, vel_desired_N );
	
	// VX- und VY-Regelung
	double ax, ay;
	pid_vx->setInput( vel_desired_N[X] - vel_measure.linear.x );	
	ax = pid_vx->getOutput();
	pid_vx->updateState( dt );

	pid_vy->setInput( vel_desired_N[Y] - vel_measure.linear.y );
	ay = pid_vy->getOutput();
	pid_vy->updateState( dt );
    
	// benötigte Neigungen berechnen
	double roll_desired = 0.0;
	double pitch_desired = 0.0;
	motion_equation( pose_measure.orientation.psi, ax, ay, roll_desired, pitch_desired );

	// Begrenzung der Winkel Phi und Theta
	if( roll_desired >  20.0/180.0*3.1415) roll_desired  = 20.0/180.0*3.1415;
	if( roll_desired <- 20.0/180.0*3.1415) roll_desired  =-20.0/180.0*3.1415;
	if( pitch_desired > 20.0/180.0*3.1415) pitch_desired = 20.0/180.0*3.1415;
	if( pitch_desired <-20.0/180.0*3.1415) pitch_desired =-20.0/180.0*3.1415;

	// Phi- und Theta-Regelung
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
	pubStell.publish(U_out);

	return true;
}



int main(int argc, char **argv)
{
 	
	ros::init(argc, argv, "Controller");

	ros::NodeHandle nh("Controller");

	// UAV-Parameter holen
	nh.getParam("m", m);

	// Parameter für Max-Geschwindigkeit holen
	nh.getParam("vx_max"	, vel_max[X]);
	nh.getParam("vy_max"	, vel_max[Y]);
	nh.getParam("vz_max"	, vel_max[Z]);
	nh.getParam("vyaw_max"	, vel_max[YAW]);

	// Parameter für Fernbedienung holen
	nh.getParam("X_Axis"	, X_Axis);
	nh.getParam("Y_Axis"	, Y_Axis);
	nh.getParam("Z_Axis"	, Z_Axis);
	nh.getParam("Yaw_Axis"	, Yaw_Axis);

	last_Prop = ros::Time::now();

	pid_vx 		= new PID( ros::NodeHandle(nh, "vxy"	) );
	pid_vy 		= new PID( ros::NodeHandle(nh, "vxy"	) );
	pid_vz 		= new PID( ros::NodeHandle(nh, "vz"		), 0, 0 );
	pid_vyaw 	= new PID( ros::NodeHandle(nh, "vyaw"	) );
	pid_roll 	= new PID( ros::NodeHandle(nh, "roll"	) );
	pid_pitch = new PID( ros::NodeHandle(nh, "pitch") );

	

	// Subscriber für die verzögerten Fernbedienungssignale
	ros::Subscriber subRC  = nh.subscribe("/rc_signal_delayed", 10, callback_rc_signal_delayed);

	// Subscriber für die Kinematikdaten (Pose + Geschwindigkeiten)
	ros::Subscriber subKin = nh.subscribe("/kin_measure", 10, callback_kin_measure);

	// Servicebereitstellung zum Ausführen der Propagation
	ros::ServiceServer service = nh.advertiseService("controller_prop", propagate);

	// Publisher für Sollgeschwindigkeiten
	pubSoll = nh.advertise<geometry_msgs::Quaternion>("/VSoll", 10);

	// Publisher für Stellgrößen, die an die Dynamic weitergegeben werden
	pubStell = nh.advertise<quadrotor_control::manipulated_variables>("/stellgroessen", 100);

	ROS_INFO( "Controller Init done" );

	ros::spin();

	return 0;
}
