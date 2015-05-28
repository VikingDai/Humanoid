#include "Kinematics.h"

Configurations::Configurations()
{
	angle_trunk_yaw.resize(TotalTimeFrame);
	angle_waist.resize(TotalTimeFrame);

	angle_left_hip_yaw.resize(TotalTimeFrame);
	angle_left_hip_roll.resize(TotalTimeFrame);
	angle_left_hip_pitch.resize(TotalTimeFrame);
	angle_legt_knee.resize(TotalTimeFrame);
	angle_left_ankle_pitch.resize(TotalTimeFrame);
	angle_left_ankle_roll.resize(TotalTimeFrame);

	angle_right_hip_yaw.resize(TotalTimeFrame);
	angle_right_hip_roll.resize(TotalTimeFrame);
	angle_right_hip_pitch.resize(TotalTimeFrame);
	angle_right_knee.resize(TotalTimeFrame);
	angle_right_ankle_pitch.resize(TotalTimeFrame);
	angle_right_ankle_roll.resize(TotalTimeFrame);
}

string SetAngleTrunkYaw( Configurations *configurations )
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
string SetAngleWaist( Configurations *configurations )
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