#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <vector>
#include <Eigen/Dense>
#include "Trajectories.h"
using namespace std;
using namespace Eigen;


const double PelvisWidth = 13.5;
const double PelvisHeight = 11;
const double ThighLength = 26;
const double ShinLength = 22;
const double AnkleHeight = 20;

struct Configuration {
	double angle_com;
	double angle_trunk_yaw;
	double angle_waist;

	double angle_left_hip_yaw;
	double angle_left_hip_roll;
	double angle_left_hip_pitch;
	double angle_legt_knee;
	double angle_left_ankle_pitch;
	double angle_left_ankle_roll;

	double angle_right_hip_yaw;
	double angle_right_hip_roll;
	double angle_right_hip_pitch;
	double angle_right_knee;
	double angle_right_ankle_pitch;
	double angle_right_ankle_roll;

	Vector3d target_com;
	Vector3d target_left_foot;
	Vector3d target_right_foot;
	
	//Vector3d actual_com;
	//Vector3d actual_left_foot;
	//Vector3d actual_right_foot;
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//constructor
	Configuration(int a);

};

typedef vector<Configuration*> Configurations;





#endif /*_KINEMATICS_H_*/