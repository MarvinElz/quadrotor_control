#ifndef CONTROLLER_H_
#define CONTROLLER_H_



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
vel  vel_measure  = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
pose pose_measure = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

ros::Time last_Prop;

PID* pid_vx;
PID* pid_vy;
PID* pid_vz;
PID* pid_vyaw;
PID* pid_roll;
PID* pid_pitch;

ros::Publisher pub;


#endif
