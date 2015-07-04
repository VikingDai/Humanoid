#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <Eigen/Dense>
#include "Trajectories.h"
#include "Util.h"
using namespace std;
using namespace Eigen;


const double PelvisWidth = 26;	//PelvisWidth = length from left hip to right hip
const double PelvisHeight = 11;	//PelvisHeignt = length form hip height to waist height
const double ThighLength = 26;
const double ShinLength = 22;
const double AnkleHeight = 20;

struct Configurations {
	VectorXd angle_trunk_yaw;
	VectorXd angle_waist;

	VectorXd angle_left_hip_yaw;
	VectorXd angle_left_hip_roll;
	VectorXd angle_left_hip_pitch;
	VectorXd angle_left_knee;
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
	Configurations( Trajectories *trajectories );
};
struct ForKinMatrix{
	VectorXd m11;
	VectorXd m12;
	VectorXd m13;
	VectorXd m14;

	VectorXd m21;
	VectorXd m22;
	VectorXd m23;
	VectorXd m24;

	VectorXd m31;
	VectorXd m32;
	VectorXd m33;
	VectorXd m34;

	VectorXd m41;
	VectorXd m42;
	VectorXd m43;
	VectorXd m44;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//constructor
	ForKinMatrix();
	ForKinMatrix(double a, double alpha, double d, VectorXd& theta);
};
struct ForKinVector{
	VectorXd v1;
	VectorXd v2;
	VectorXd v3;
	VectorXd v4;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//constructor
	ForKinVector();
	ForKinVector(MatrixXd& mat);
};

string DesignAngleTrunkYaw( Configurations *configurations );
string DesignAngleWaist( Configurations *configurations );
string LowerBodyInvKin( Configurations *configurations, Trajectories *trajectories );
void CalAngleHipYaw( Configurations *configurations, Trajectories *trajectories );
void CalAllLegAngle( Configurations *configurations, Trajectories *trajectories, ForKinMatrix *T_left_do, ForKinMatrix *T_right_do );
void CalLegAngle(VectorXd *angle_hip_roll, VectorXd *angle_hip_pitch, VectorXd *angle_knee, ForKinVector* fk_ankle_new );
void ForKinMatrixMultiplyMatrix( ForKinMatrix *mat, ForKinMatrix *mat1, ForKinMatrix *mat2);
void ForKinMatrixMultiplyVector( ForKinVector* vec_result, ForKinMatrix* mat, ForKinVector* vec);
void ForKinMatrixInverse( ForKinMatrix *mat_inv, ForKinMatrix *mat );
VectorXd VectorBitwiseMultiply( VectorXd &vec1, VectorXd &vec2);
VectorXd SinVector( VectorXd& theta);
VectorXd CosVector( VectorXd& theta);
void GetArcSinVector( VectorXd& arc_sin_vec, VectorXd& vec);
void ConfigurationWriteFiles( Configurations *configurations );



#endif /*_KINEMATICS_H_*/