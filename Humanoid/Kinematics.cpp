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
ForKinMatrix:: ForKinMatrix()
{

}
ForKinMatrix:: ForKinMatrix(double a, double alpha, double d, VectorXd& theta)
{
	int length = theta.size();
	VectorXd zeros_vec = VectorXd::Zero(theta.size());
	VectorXd ones_vec  = VectorXd::Ones(theta.size());
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
	vector <ForKinMatrix*> T_left(8);
	vector <ForKinMatrix*> T_right(8);

	CalAngleHipYaw( configurations, trajectories );

	T_left[0] =											new ForKinMatrix(	0,					0,		0,	trajectories->com_direction			);
	T_left[1] = ForKinMatrixMultiplyMatrix( T_left[0],	new ForKinMatrix(	0,					0,		0,	configurations->angle_trunk_yaw)	);
	T_left[2] = ForKinMatrixMultiplyMatrix( T_left[1],	new ForKinMatrix(	0,					pi/2,	0,	configurations->angle_waist)		);
	T_left[3] = ForKinMatrixMultiplyMatrix( T_left[2],	new ForKinMatrix(	PelvisWidth/2,		pi/2,	0,	configurations->angle_left_hip_yaw) );
	
	T_right[0] =										new ForKinMatrix(	0,					0,		0,	trajectories->com_direction			);
	T_right[1] = ForKinMatrixMultiplyMatrix( T_left[0],	new ForKinMatrix(	0,					0,		0,	configurations->angle_trunk_yaw)	);
	T_right[2] = ForKinMatrixMultiplyMatrix( T_left[1],	new ForKinMatrix(	0,					pi/2,	0,	configurations->angle_waist)		);
	T_right[3] = ForKinMatrixMultiplyMatrix( T_left[2],	new ForKinMatrix(	-PelvisWidth/2,		pi/2,	0,	configurations->angle_left_hip_yaw) );

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
ForKinMatrix* ForKinMatrixMultiplyMatrix( ForKinMatrix *mat1, ForKinMatrix *mat2)
{
	ForKinMatrix *mat = new ForKinMatrix;
	mat->m11 =  VectorBitwiseMultiply(mat1->m11, mat2->m11) + VectorBitwiseMultiply(mat1->m12, mat2->m21) + VectorBitwiseMultiply(mat1->m13, mat2->m31) + VectorBitwiseMultiply(mat1->m14, mat2->m41);
	mat->m12 =  VectorBitwiseMultiply(mat1->m11, mat2->m12) + VectorBitwiseMultiply(mat1->m12, mat2->m22) + VectorBitwiseMultiply(mat1->m13, mat2->m32) + VectorBitwiseMultiply(mat1->m14, mat2->m42);
	mat->m13 =  VectorBitwiseMultiply(mat1->m11, mat2->m13) + VectorBitwiseMultiply(mat1->m12, mat2->m23) + VectorBitwiseMultiply(mat1->m13, mat2->m33) + VectorBitwiseMultiply(mat1->m14, mat2->m43);
	mat->m14 =  VectorBitwiseMultiply(mat1->m11, mat2->m14) + VectorBitwiseMultiply(mat1->m12, mat2->m24) + VectorBitwiseMultiply(mat1->m13, mat2->m34) + VectorBitwiseMultiply(mat1->m14, mat2->m44);

	mat->m21 =  VectorBitwiseMultiply(mat1->m21, mat2->m11) + VectorBitwiseMultiply(mat1->m22, mat2->m21) + VectorBitwiseMultiply(mat1->m23, mat2->m31) + VectorBitwiseMultiply(mat1->m24, mat2->m41);
	mat->m22 =  VectorBitwiseMultiply(mat1->m21, mat2->m12) + VectorBitwiseMultiply(mat1->m22, mat2->m22) + VectorBitwiseMultiply(mat1->m23, mat2->m32) + VectorBitwiseMultiply(mat1->m24, mat2->m42);
	mat->m23 =  VectorBitwiseMultiply(mat1->m21, mat2->m13) + VectorBitwiseMultiply(mat1->m22, mat2->m23) + VectorBitwiseMultiply(mat1->m23, mat2->m33) + VectorBitwiseMultiply(mat1->m24, mat2->m43);
	mat->m24 =  VectorBitwiseMultiply(mat1->m21, mat2->m14) + VectorBitwiseMultiply(mat1->m22, mat2->m24) + VectorBitwiseMultiply(mat1->m23, mat2->m34) + VectorBitwiseMultiply(mat1->m24, mat2->m44);

	mat->m31 =  VectorBitwiseMultiply(mat1->m31, mat2->m11) + VectorBitwiseMultiply(mat1->m32, mat2->m21) + VectorBitwiseMultiply(mat1->m33, mat2->m31) + VectorBitwiseMultiply(mat1->m34, mat2->m41);
	mat->m32 =  VectorBitwiseMultiply(mat1->m31, mat2->m12) + VectorBitwiseMultiply(mat1->m32, mat2->m22) + VectorBitwiseMultiply(mat1->m33, mat2->m32) + VectorBitwiseMultiply(mat1->m34, mat2->m42);
	mat->m33 =  VectorBitwiseMultiply(mat1->m31, mat2->m13) + VectorBitwiseMultiply(mat1->m32, mat2->m23) + VectorBitwiseMultiply(mat1->m33, mat2->m33) + VectorBitwiseMultiply(mat1->m34, mat2->m43);
	mat->m34 =  VectorBitwiseMultiply(mat1->m31, mat2->m14) + VectorBitwiseMultiply(mat1->m32, mat2->m24) + VectorBitwiseMultiply(mat1->m33, mat2->m34) + VectorBitwiseMultiply(mat1->m34, mat2->m44);

	mat->m41 =  VectorBitwiseMultiply(mat1->m41, mat2->m11) + VectorBitwiseMultiply(mat1->m42, mat2->m21) + VectorBitwiseMultiply(mat1->m43, mat2->m31) + VectorBitwiseMultiply(mat1->m44, mat2->m41);
	mat->m42 =  VectorBitwiseMultiply(mat1->m41, mat2->m12) + VectorBitwiseMultiply(mat1->m42, mat2->m22) + VectorBitwiseMultiply(mat1->m43, mat2->m32) + VectorBitwiseMultiply(mat1->m44, mat2->m42);
	mat->m43 =  VectorBitwiseMultiply(mat1->m41, mat2->m13) + VectorBitwiseMultiply(mat1->m42, mat2->m23) + VectorBitwiseMultiply(mat1->m43, mat2->m33) + VectorBitwiseMultiply(mat1->m44, mat2->m43);
	mat->m44 =  VectorBitwiseMultiply(mat1->m41, mat2->m14) + VectorBitwiseMultiply(mat1->m42, mat2->m24) + VectorBitwiseMultiply(mat1->m43, mat2->m34) + VectorBitwiseMultiply(mat1->m44, mat2->m44);

	return mat;
}
//ForKinVector ForKinMatrixMultiplyVector( ForKinMatrix *mat, ForKinMatrix *vec)
//{
//
//}
VectorXd VectorBitwiseMultiply( VectorXd &vec1, VectorXd &vec2)
{
	VectorXd vec(vec1.size());
	for(int i=0; i<vec1.size(); i++)
		vec[i] = vec1[i]*vec2[i];

	return vec;
}
VectorXd CosVector( VectorXd& theta)
{
	VectorXd m(theta.size());
	for(int i=0; i<theta.size(); i++)
		m[i] = cos(theta[i]);

	return m;
}
VectorXd SinVector( VectorXd& theta)
{
	VectorXd m(theta.size());
	for(int i=0; i<theta.size(); i++)
		m[i] = sin(theta[i]);

	return m;
}

