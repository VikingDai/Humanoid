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
ForKinMatrix::ForKinMatrix()
{

}
ForKinMatrix::ForKinMatrix(double a, double alpha, double d, VectorXd& theta)
{
	int length = theta.size();
	VectorXd zeros_vec = VectorXd::Zero(theta.size());
	VectorXd ones_vec  = VectorXd::Ones(theta.size());
	VectorXd sin_theta_vec = SinVector(theta);
	VectorXd cos_theta_vec = CosVector(theta);

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
ForKinVector::ForKinVector()
{


}
ForKinVector::ForKinVector(MatrixXd& mat)
{
	int length = mat.row(0).size();

	v1 = mat.row(0);
	v2 = mat.row(1);
	v3 = mat.row(2);
	v4 = VectorXd::Zero(length);
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
	ForKinMatrix  T_left_oa,  T_left_ob,  T_left_oc,  T_left_od,  T_left_oe,  T_left_of,  T_left_og,  T_left_oh,  T_left_oi;
	ForKinMatrix T_right_oa, T_right_ob, T_right_oc, T_right_od, T_right_oe, T_right_of, T_right_og, T_right_oh, T_right_oi;

	CalAngleHipYaw( configurations, trajectories );

	T_left_oa =											  ForKinMatrix(	0,					0,		0,	trajectories->com_direction			);
	ForKinMatrixMultiplyMatrix( &T_left_ob, &T_left_oa,	 &ForKinMatrix(	0,					0,		0,	configurations->angle_trunk_yaw)	);
	ForKinMatrixMultiplyMatrix( &T_left_oc, &T_left_ob,	 &ForKinMatrix(	0,					pi/2,	0,	configurations->angle_waist)		);
	ForKinMatrixMultiplyMatrix( &T_left_od, &T_left_oc,	 &ForKinMatrix(	PelvisWidth/2,		pi/2,	0,	configurations->angle_left_hip_yaw) );
	
	T_right_oa =											 ForKinMatrix(	0,					0,		0,	trajectories->com_direction			);
	ForKinMatrixMultiplyMatrix( &T_right_ob, &T_right_oa,	&ForKinMatrix(	0,					0,		0,	configurations->angle_trunk_yaw)	);
	ForKinMatrixMultiplyMatrix( &T_right_oc, &T_right_ob,	&ForKinMatrix(	0,					pi/2,	0,	configurations->angle_waist)		);
	ForKinMatrixMultiplyMatrix( &T_right_od, &T_right_oc,	&ForKinMatrix(	-PelvisWidth/2,		pi/2,	0,	configurations->angle_left_hip_yaw) );


	CalAllLegAngle( configurations, trajectories, &T_left_od, &T_right_od );


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
void CalAllLegAngle( Configurations *configurations, Trajectories *trajectories, ForKinMatrix *T_left_od, ForKinMatrix *T_right_od )
{
	ForKinMatrix T_left_do, T_right_do;
	ForKinMatrixInverse(&T_left_do, T_left_od );
	ForKinMatrixInverse(&T_right_do, T_right_od);


	MatrixXd ankle_height = MatrixXd::Zero(3, trajectories->TotalTimeFrame);
	ankle_height.row(2).setConstant(AnkleHeight);

	MatrixXd left_ankle  = trajectories->left_foot  + ankle_height - trajectories->com;
	MatrixXd right_ankle = trajectories->right_foot + ankle_height - trajectories->com;

	ForKinVector fk_left_ankle_new, fk_right_ankle_new; 
	ForKinMatrixMultiplyVector(&fk_left_ankle_new, &T_left_do,  &ForKinVector(left_ankle));
	ForKinMatrixMultiplyVector(&fk_right_ankle_new, &T_right_do, &ForKinVector(right_ankle));

	CalLegAngle(&configurations->angle_left_hip_roll, &configurations->angle_left_hip_pitch, &configurations->angle_left_knee, &fk_left_ankle_new);
	CalLegAngle(&configurations->angle_right_hip_roll, &configurations->angle_right_hip_pitch, &configurations->angle_right_knee, &fk_right_ankle_new);





}
void CalLegAngle(VectorXd *angle_hip_roll, VectorXd *angle_hip_pitch, VectorXd *angle_knee, ForKinVector* fk_ankle_new )
{
	int vec_length = angle_hip_roll->size();
	double hip_to_ankle_length;

	for(int i=0; i<vec_length; i++){
		// length of virtual extensible leg
		hip_to_ankle_length = sqrt(pow(fk_ankle_new->v1(i), 2) + pow(fk_ankle_new->v2(i), 2) + pow(fk_ankle_new->v3(i), 2));

		// IK of virtual extensible leg
		(*angle_hip_pitch)(i) = asin(-1 * fk_ankle_new->v2(i) / hip_to_ankle_length);
		double a = (*angle_hip_pitch)(i);
		(*angle_hip_roll)(i) = asin(fk_ankle_new->v1(i) / hip_to_ankle_length / cos((*angle_hip_pitch)(i)));

		// IK of real leg corrected by law of cosine 
		(*angle_hip_pitch)(i) -= acos((pow(ThighLength, 2) + pow(hip_to_ankle_length, 2) - pow(ShinLength, 2)) / (2 * ThighLength * hip_to_ankle_length));
		double b =  acos((pow(ThighLength, 2) + pow(hip_to_ankle_length, 2) - pow(ShinLength, 2)) / (2 * ThighLength * hip_to_ankle_length));
		(*angle_knee)(i) = acos((pow(ThighLength, 2) + pow(ShinLength, 2) - pow(hip_to_ankle_length, 2)) / (2 * ThighLength * ShinLength));

		if(i==0){
			cout << fk_ankle_new->v1(i) << " " << fk_ankle_new->v2(i) << " " << fk_ankle_new->v3(i) << endl;
			cout << hip_to_ankle_length << endl;
			cout << pow(ThighLength, 2) << " " << pow(hip_to_ankle_length, 2) << " " << pow(ShinLength, 2) << " " << (2 * ThighLength * hip_to_ankle_length) << endl;
			cout << (pow(ThighLength, 2) + pow(hip_to_ankle_length, 2) - pow(ShinLength, 2)) << " " << (2 * ThighLength * hip_to_ankle_length) << endl;
			cout << a << " " << b << " " << (*angle_hip_pitch)(i) << " " << (*angle_hip_roll)(i) << " " << (*angle_knee)(i) << endl;
			cout << endl;
		}
	}
	
}
void ForKinMatrixMultiplyMatrix( ForKinMatrix *mat, ForKinMatrix *mat1, ForKinMatrix *mat2)
{
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

}
void ForKinMatrixMultiplyVector( ForKinVector* vec_result, ForKinMatrix* mat, ForKinVector* vec)
{
	vec_result->v1 = VectorBitwiseMultiply(mat->m11, vec->v1) + VectorBitwiseMultiply(mat->m12, vec->v2) + VectorBitwiseMultiply(mat->m13, vec->v3) + VectorBitwiseMultiply(mat->m14, vec->v4);
	vec_result->v2 = VectorBitwiseMultiply(mat->m21, vec->v1) + VectorBitwiseMultiply(mat->m22, vec->v2) + VectorBitwiseMultiply(mat->m23, vec->v3) + VectorBitwiseMultiply(mat->m24, vec->v4);
	vec_result->v3 = VectorBitwiseMultiply(mat->m31, vec->v1) + VectorBitwiseMultiply(mat->m32, vec->v2) + VectorBitwiseMultiply(mat->m33, vec->v3) + VectorBitwiseMultiply(mat->m34, vec->v4);
	vec_result->v4 = VectorBitwiseMultiply(mat->m41, vec->v1) + VectorBitwiseMultiply(mat->m42, vec->v2) + VectorBitwiseMultiply(mat->m43, vec->v3) + VectorBitwiseMultiply(mat->m44, vec->v4);
}
void ForKinMatrixInverse( ForKinMatrix *mat_inv, ForKinMatrix *mat )
{
	mat_inv->m11 = mat->m11;
	mat_inv->m12 = mat->m21;
	mat_inv->m13 = mat->m31;

	mat_inv->m21 = mat->m12;
	mat_inv->m22 = mat->m22;
	mat_inv->m23 = mat->m32;
	
	mat_inv->m31 = mat->m13;
	mat_inv->m32 = mat->m23;
	mat_inv->m33 = mat->m33;

	mat_inv->m14 = -1 * VectorBitwiseMultiply(mat_inv->m11, mat->m14) + -1 * VectorBitwiseMultiply(mat_inv->m12, mat->m24) + -1 * VectorBitwiseMultiply(mat_inv->m13, mat->m34);
	mat_inv->m24 = -1 * VectorBitwiseMultiply(mat_inv->m21, mat->m14) + -1 * VectorBitwiseMultiply(mat_inv->m22, mat->m24) + -1 * VectorBitwiseMultiply(mat_inv->m23, mat->m34);
	mat_inv->m34 = -1 * VectorBitwiseMultiply(mat_inv->m31, mat->m14) + -1 * VectorBitwiseMultiply(mat_inv->m32, mat->m24) + -1 * VectorBitwiseMultiply(mat_inv->m33, mat->m34);
	
	mat_inv->m41 = mat->m41;
	mat_inv->m42 = mat->m42;
	mat_inv->m43 = mat->m43;
	mat_inv->m44 = mat->m44;
}
VectorXd VectorBitwiseMultiply( VectorXd &vec1, VectorXd &vec2)
{
	VectorXd vec(vec1.size());
	for(int i=0; i<vec1.size(); i++)
		vec[i] = vec1[i]*vec2[i];

	return vec;
}
VectorXd SinVector( VectorXd& theta)
{
	VectorXd m(theta.size());
	for(int i=0; i<theta.size(); i++)
		m[i] = sin(theta[i]);

	return m;
}
VectorXd CosVector( VectorXd& theta)
{
	VectorXd m(theta.size());
	for(int i=0; i<theta.size(); i++)
		m[i] = cos(theta[i]);

	return m;
}
void GetArcSinVector( VectorXd& arc_sin_vec, VectorXd& vec)
{
	int length = vec.size();
	arc_sin_vec.resize(length);

	for(int i=0; i<length; i++){
		arc_sin_vec[i] = asin(vec[i]);
	}
}

void ConfigurationWriteFiles( Configurations *configurations )
{
	cout << "Writing configuration files..." << endl;

	EigenWriteFile(configurations->angle_left_hip_roll, "angle_left_hip_roll", WriteFilePath);
	EigenWriteFile(configurations->angle_left_hip_pitch, "angle_left_hip_pitch", WriteFilePath);
	EigenWriteFile(configurations->angle_left_knee, "angle_left_knee", WriteFilePath);

	EigenWriteFile(configurations->angle_right_hip_roll, "angle_right_hip_roll", WriteFilePath);
	EigenWriteFile(configurations->angle_right_hip_pitch, "angle_right_hip_pitch", WriteFilePath);
	EigenWriteFile(configurations->angle_right_knee, "angle_right_knee", WriteFilePath);

}