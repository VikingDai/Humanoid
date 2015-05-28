#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <Eigen/Dense>
#include "Trajectories.h"
using namespace std;
using namespace Eigen;


const double PelvisWidth = 13.5;
const double PelvisHeight = 11;
const double ThighLength = 26;
const double ShinLength = 22;
const double AnkleHeight = 20;

struct Configurations {
	VectorXd angle_trunk_yaw;
	VectorXd angle_waist;

	VectorXd angle_left_hip_yaw;
	VectorXd angle_left_hip_roll;
	VectorXd angle_left_hip_pitch;
	VectorXd angle_legt_knee;
	VectorXd angle_left_ankle_pitch;
	VectorXd angle_left_ankle_roll;

	VectorXd angle_right_hip_yaw;
	VectorXd angle_right_hip_roll;
	VectorXd angle_right_hip_pitch;
	VectorXd angle_right_knee;
	VectorXd angle_right_ankle_pitch;
	VectorXd angle_right_ankle_roll;

	//MatrixXd target_com;
	//MatrixXd target_left_foot;
	//MatrixXd target_right_foot;
	
	//MatrixXd actual_com;
	//MatrixXd actual_left_foot;
	//MatrixXd actual_right_foot;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//constructor
	Configurations();
};

string SetAngleTrunkYaw( Configurations *configurations );
string SetAngleWaist( Configurations *configurations );



#endif /*_KINEMATICS_H_*/