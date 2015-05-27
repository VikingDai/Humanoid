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