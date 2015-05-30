#include "Kinematics.h"

Configurations::Configurations( Trajectories *trajectories )
{
	angle_trunk_yaw			= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_waist				= VectorXd::Zero(trajectories->TotalTimeFrame);

	angle_left_hip_yaw		= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_left_hip_roll		= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_left_hip_pitch	= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_left_knee			= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_left_ankle_pitch	= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_left_ankle_roll	= VectorXd::Zero(trajectories->TotalTimeFrame);

	angle_right_hip_yaw		= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_right_hip_roll	= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_right_hip_pitch	= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_right_knee		= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_right_ankle_pitch	= VectorXd::Zero(trajectories->TotalTimeFrame);
	angle_right_ankle_roll	= VectorXd::Zero(trajectories->TotalTimeFrame);
}
ForKinMatrix:: ForKinMatrix(double a, double alpha, double d, VectorXd *theta)
{
	int length = theta->size();
	VectorXd zeros_vec = VectorXd::Zero(theta->size());
	VectorXd ones_vec  = VectorXd::Ones(theta->size());
	VectorXd cos_theta_vec = CosVector(theta);
	VectorXd sin_theta_vec = SinVector(theta);

	m11 = cos_theta_vec;
	m12 = -1 * sin_theta_vec;
	m13 = zeros_vec;
	m14 = a * ones_vec;

	m21 = cos(alpha) * sin_theta_vec;
	m22 = cos(alpha) * cos_theta_vec;
	m23 = -1 * sin(alpha) * ones_vec;
	m24 = -1 * d * sin(alpha) * ones_vec;

	m31 = sin(alpha) * sin_theta_vec;
	m32 = sin(alpha) * cos_theta_vec;
	m33 = cos(alpha) * ones_vec;
	m34 = d * cos(alpha) * ones_vec;

	m41 = zeros_vec;
	m42 = zeros_vec;
	m43 = zeros_vec;
	m44 = ones_vec;
}

string DesignAngleTrunkYaw( Configurations *configurations )
{
	string err;
	cout << "Calculating angle_trunk_yaw trajectory..." << endl;

	// temp acceptable solution for angle_trunk_yaw 
	configurations->angle_trunk_yaw.setZero();

	// angle_trunk_yaw trajectory legal test
	if(0){
	// TODO
	// check if the angle_trunk_yaw trajectory generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
string DesignAngleWaist( Configurations *configurations )
{
	string err;
	cout << "Calculating angle_waist trajectory..." << endl;

	// temp acceptable solution for angle_waist 
	configurations->angle_waist.setZero();

	// angle_waist trajectory legal test
	if(0){
	// TODO
	// check if the angle_waist trajectory generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
string LowerBodyInvKin( Configurations *configurations, Trajectories *trajectories )
{
	// TODO: Given com_direction, foot_direction, angle_trunk_yaw, angle_waist
	//		 Calculate all lower body angle
	string err;
	cout << "Calculating all lower body angle trajectory..." << endl;

	CalAngleHipYaw( configurations, trajectories );

	ForKinMatrix T_oa_left = ForKinMatrix(0, 0, 0, &(trajectories->com_direction));
	ForKinMatrix T_ab_left = ForKinMatrix(0, 0, 0, &(configurations->angle_trunk_yaw));
	ForKinMatrix T_bc_left = ForKinMatrix(0, 0, 0, &(configurations->angle_waist));
	ForKinMatrix T_cd_left = ForKinMatrix(0, 0, 0, &(configurations->angle_left_hip_yaw));



	// all lower body angle trajectory legal test
	if(0){
	// TODO
	// check if all lower body angle trajectory generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;


}
void CalAngleHipYaw( Configurations *configurations, Trajectories *trajectories )
{
	configurations->angle_left_hip_yaw = trajectories->com_direction - trajectories->left_foot_direction - configurations->angle_trunk_yaw;
	configurations->angle_right_hip_yaw = trajectories->com_direction - trajectories->right_foot_direction - configurations->angle_trunk_yaw;

}
//VectorXd VectorBitwiseMultiply( VectorXd *v1, VectorXd *v2)
//{
//	VectorXd v(v1->size());
//	for(int i=0; i<v1->size(); i++)
//		v[i] = v1[i]*v2[i];
//
//	return v;
//}
VectorXd CosVector( VectorXd *theta)
{
	VectorXd m(theta->size());
	for(int i=0; i<theta->size(); i++)
		m[i] = cos((*theta)[i]);

	return m;
}
VectorXd SinVector( VectorXd *theta)
{
	VectorXd m(theta->size());
	for(int i=0; i<theta->size(); i++)
		m[i] = sin((*theta)[i]);

	return m;
}

