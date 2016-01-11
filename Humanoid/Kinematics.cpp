#include "Kinematics.h"

Configurations::Configurations( Trajectories *trajectories )
{
	TotalTimeFrame			= trajectories->TotalTimeFrame;

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
//ForKinMatrix::ForKinMatrix()
//{
//
//}
//ForKinMatrix::ForKinMatrix(double a, double alpha, double d, VectorXd& theta)
//{
//	int length = theta.size();
//	VectorXd zeros_vec = VectorXd::Zero(theta.size());
//	VectorXd ones_vec  = VectorXd::Ones(theta.size());
//	VectorXd sin_theta_vec = SinVector(theta);
//	VectorXd cos_theta_vec = CosVector(theta);
//
//	m11 = cos_theta_vec;
//	m12 = -1 * sin_theta_vec;
//	m13 = zeros_vec;
//	m14 = a * ones_vec;
//
//	m21 = cos(alpha) * sin_theta_vec;
//	m22 = cos(alpha) * cos_theta_vec;
//	m23 = -1 * sin(alpha) * ones_vec;
//	m24 = -1 * d * sin(alpha) * ones_vec;
//
//	m31 = sin(alpha) * sin_theta_vec;
//	m32 = sin(alpha) * cos_theta_vec;
//	m33 = cos(alpha) * ones_vec;
//	m34 = d * cos(alpha) * ones_vec;
//
//	m41 = zeros_vec;
//	m42 = zeros_vec;
//	m43 = zeros_vec;
//	m44 = ones_vec;
//}
//ForKinVector::ForKinVector()
//{
//
//
//}
//ForKinVector::ForKinVector(MatrixXd& mat)
//{
//	int length = mat.row(0).size();
//
//	v1 = mat.row(0);
//	v2 = mat.row(1);
//	v3 = mat.row(2);
//	v4 = VectorXd::Zero(length);
//}
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


	//ForKinMatrix  T_left_oa,  T_left_ob,  T_left_oc,  T_left_od,  T_left_oe,  T_left_of,  T_left_og,  T_left_oh,  T_left_oi;
	//ForKinMatrix T_right_oa, T_right_ob, T_right_oc, T_right_od, T_right_oe, T_right_of, T_right_og, T_right_oh, T_right_oi;

	CalAngleHipYaw( configurations, trajectories );


	Matrix4d  T_left_oa,  T_left_ab,  T_left_bc,  T_left_cd,  T_left_de,  T_left_ef,  T_left_fg,  T_left_gh,  T_left_hi,
			  T_right_oa, T_right_ab, T_right_bc, T_right_cd, T_right_de, T_right_ef, T_right_fg, T_right_gh, T_right_hi,
			  T_left_od, T_left_do, T_right_od, T_right_do,
			  T_left_oh, T_left_ho, T_right_oh, T_right_ho;

	MatrixXd FK_left_ankle(4,1), FK_right_ankle(4,1),
			 FK_left_ankle_new(4,1), FK_right_ankle_new(4,1),
			 FK_left_sole(4,1), FK_right_sole(4,1),
			 FK_left_sole_new(4,1), FK_right_sole_new(4,1);

	for(int i=0; i<trajectories->TotalTimeFrame; i++){
		
		// Caculation of hip and knee angle
		ForKinMatrix(&T_left_oa,	0,				0,		0,				trajectories->com_direction(i)			);
		ForKinMatrix(&T_left_ab, 	0,				0,		0,				configurations->angle_trunk_yaw(i)+pi/2	);
		ForKinMatrix(&T_left_bc, 	0,				pi/2,	0,				configurations->angle_waist(i)			);
		ForKinMatrix(&T_left_cd, 	PelvisWidth/2,	pi/2,	PelvisHeight,	configurations->angle_left_hip_yaw(i)	);

		ForKinMatrix(&T_right_oa, 	0,				0,		0,				trajectories->com_direction(i)			);
		ForKinMatrix(&T_right_ab, 	0,				0,		0,				configurations->angle_trunk_yaw(i)+pi/2	);
		ForKinMatrix(&T_right_bc, 	0,				pi/2,	0,				configurations->angle_waist(i)			);
		ForKinMatrix(&T_right_cd, 	-PelvisWidth/2,	pi/2,	PelvisHeight,	configurations->angle_right_hip_yaw(i)	);

		T_left_od  = T_left_oa  * T_left_ab  * T_left_bc  * T_left_cd;
		T_right_od = T_right_oa * T_right_ab * T_right_bc * T_right_cd;

		ForKinMatrixInverse(&T_left_do,  &T_left_od);
		ForKinMatrixInverse(&T_right_do, &T_right_od);

		FK_left_ankle << trajectories->left_foot(0,i) - trajectories->com(0,i),
						 trajectories->left_foot(1,i) - trajectories->com(1,i),
						 trajectories->left_foot(2,i) - trajectories->com(2,i) + AnkleHeight,
						 1;
		FK_right_ankle << trajectories->right_foot(0,i) - trajectories->com(0,i),
						  trajectories->right_foot(1,i) - trajectories->com(1,i),
						  trajectories->right_foot(2,i) - trajectories->com(2,i) + AnkleHeight,
						  1;

		FK_left_ankle_new  =  T_left_do  * FK_left_ankle;
		FK_right_ankle_new =  T_right_do * FK_right_ankle;

		CalLegAngle( configurations, i, &FK_left_ankle_new, &FK_right_ankle_new);


		// Caculation of ankle angle
		ForKinMatrix(&T_left_de,	0,				-pi/2,		0,		configurations->angle_left_hip_roll(i)-pi/2	);
		ForKinMatrix(&T_left_ef, 	0,				-pi/2,		0,		configurations->angle_left_hip_pitch(i)		);
		ForKinMatrix(&T_left_fg, 	ThighLength,		0,		0,		configurations->angle_left_knee(i)			);
		ForKinMatrix(&T_left_gh, 	ShinLength,			0,		0,		0											);

		ForKinMatrix(&T_right_de,	0,				-pi/2,		0,		configurations->angle_right_hip_roll(i)-pi/2);
		ForKinMatrix(&T_right_ef, 	0,				-pi/2,		0,		configurations->angle_right_hip_pitch(i)	);
		ForKinMatrix(&T_right_fg, 	ThighLength,		0,		0,		configurations->angle_right_knee(i)			);
		ForKinMatrix(&T_right_gh, 	ShinLength,			0,		0,		0											);

		T_left_oh  = T_left_od  * T_left_de  * T_left_ef  * T_left_fg  * T_left_gh;
		T_right_oh = T_right_od * T_right_de * T_right_ef * T_right_fg * T_right_gh;

		ForKinMatrixInverse(&T_left_ho,  &T_left_oh);
		ForKinMatrixInverse(&T_right_ho, &T_right_oh);

		FK_left_sole << trajectories->left_foot(0,i) - trajectories->com(0,i),
						trajectories->left_foot(1,i) - trajectories->com(1,i),
						trajectories->left_foot(2,i) - trajectories->com(2,i),
						1;
		FK_right_sole << trajectories->right_foot(0,i) - trajectories->com(0,i),
						 trajectories->right_foot(1,i) - trajectories->com(1,i),
						 trajectories->right_foot(2,i) - trajectories->com(2,i),
						 1;

		FK_left_sole_new   =  T_left_ho   * FK_left_sole;
		FK_right_sole_new  =  T_right_ho  * FK_right_sole;

		CalAnkleAngle( configurations, i, &FK_left_sole_new, &FK_right_sole_new);


	}


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
void ForKinMatrix(Matrix4d* mat, double a, double alpha, double d, double theta)
{
	*mat << cos(theta),				-1*sin(theta),			0,				a,
			cos(alpha)*sin(theta),	cos(alpha)*cos(theta),	-1*sin(alpha),	-1*d*sin(alpha),
			sin(alpha)*sin(theta),	sin(alpha)*cos(theta),	cos(alpha),		d*cos(alpha),
			0,						0,						0,				1;
}
void ForKinMatrixInverse(Matrix4d* mat_inv, Matrix4d* mat)
{
	Matrix3d R = (*mat).block(0,0,3,3);
	MatrixXd P = (*mat).block(0,3,3,1);

	(*mat_inv).block(0,0,3,3) = R.transpose();
	(*mat_inv).block(0,3,3,1) = -1 * R.transpose() * P;
	(*mat_inv).block(3,0,1,4) =  (*mat).block(3,0,1,4);
}
void CalLegAngle(Configurations *configurations, int i, MatrixXd *left_ankle, MatrixXd *right_ankle)
{
	double left_hip_to_ankle_length, right_hip_to_ankle_length;

	// left foot
	// length of virtual extensible leg
	left_hip_to_ankle_length = sqrt(pow((*left_ankle)(0), 2) + pow((*left_ankle)(1), 2) + pow((*left_ankle)(2), 2));

	// IK of virtual extensible leg
	configurations->angle_left_hip_pitch(i) = asin(-1 * (*left_ankle)(1) / left_hip_to_ankle_length);
	configurations->angle_left_hip_roll(i) = asin((*left_ankle)(0) / left_hip_to_ankle_length / cos(configurations->angle_left_hip_pitch(i)));

	// IK of real leg corrected by law of cosine 
	configurations->angle_left_hip_pitch(i) -= acos((pow(ThighLength, 2) + pow(left_hip_to_ankle_length, 2) - pow(ShinLength, 2)) / (2 * ThighLength * left_hip_to_ankle_length));
	configurations->angle_left_knee(i) = pi - acos((pow(ThighLength, 2) + pow(ShinLength, 2) - pow(left_hip_to_ankle_length, 2)) / (2 * ThighLength * ShinLength));

	
	// right foot
	// length of virtual extensible leg
	right_hip_to_ankle_length = sqrt(pow((*right_ankle)(0), 2) + pow((*right_ankle)(1), 2) + pow((*right_ankle)(2), 2));

	// IK of virtual extensible leg
	configurations->angle_right_hip_pitch(i) = asin(-1 * (*right_ankle)(1) / right_hip_to_ankle_length);
	configurations->angle_right_hip_roll(i) = asin((*right_ankle)(0) / right_hip_to_ankle_length / cos(configurations->angle_right_hip_pitch(i)));

	// IK of real leg corrected by law of cosine 
	configurations->angle_right_hip_pitch(i) -= acos((pow(ThighLength, 2) + pow(right_hip_to_ankle_length, 2) - pow(ShinLength, 2)) / (2 * ThighLength * right_hip_to_ankle_length));
	configurations->angle_right_knee(i) = pi - acos((pow(ThighLength, 2) + pow(ShinLength, 2) - pow(right_hip_to_ankle_length, 2)) / (2 * ThighLength * ShinLength));
}
void CalAnkleAngle(Configurations *configurations, int i, MatrixXd *left_sole, MatrixXd *right_sole)
{
	// left foot
	configurations->angle_left_ankle_roll(i) = asin((*left_sole)(2) / AnkleHeight);
	configurations->angle_left_ankle_pitch(i) = asin((*left_sole)(1) / cos(configurations->angle_left_ankle_roll(i)) / AnkleHeight);	
	
	// right foot
	configurations->angle_right_ankle_roll(i) = asin((*right_sole)(2) / AnkleHeight);
	configurations->angle_right_ankle_pitch(i) = asin((*right_sole)(1) / cos(configurations->angle_right_ankle_roll(i)) / AnkleHeight);
}

void ConfigurationWriteFiles( Configurations *configurations )
{
	cout << "Writing configuration files..." << endl;

	EigenWriteFile(configurations->angle_left_hip_roll, "angle_left_hip_roll", WriteFilePath);
	EigenWriteFile(configurations->angle_left_hip_pitch, "angle_left_hip_pitch", WriteFilePath);
	EigenWriteFile(configurations->angle_left_knee, "angle_left_knee", WriteFilePath);
	EigenWriteFile(configurations->angle_left_ankle_roll, "angle_left_ankle_roll", WriteFilePath);
	EigenWriteFile(configurations->angle_left_ankle_pitch, "angle_left_ankle_pitch", WriteFilePath);

	EigenWriteFile(configurations->angle_right_hip_roll, "angle_right_hip_roll", WriteFilePath);
	EigenWriteFile(configurations->angle_right_hip_pitch, "angle_right_hip_pitch", WriteFilePath);
	EigenWriteFile(configurations->angle_right_knee, "angle_right_knee", WriteFilePath);
	EigenWriteFile(configurations->angle_right_ankle_roll, "angle_right_ankle_roll", WriteFilePath);
	EigenWriteFile(configurations->angle_right_ankle_pitch, "angle_right_ankle_pitch", WriteFilePath);
}