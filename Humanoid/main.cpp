#include <iostream>
#include <vector>
#include "Steps.h"
#include "Trajectories.h"
#include "Kinematics.h"
using namespace std;

void main()
{	
	//Path Planning
	Steps steps;

	SetFootsteps( &steps );
	ReviseStepDirection( &steps );
	SetAllCP( &steps );
	SetAllZmp( &steps );


	//Trajectory Calculation
	Trajectories trajectories( &steps );
	
	SetZmpTrajectory( &trajectories, &steps );
	SetFootTrajectory( &trajectories, &steps );
	SetFootDirection( &trajectories, &steps );
	SetComTrajectory( &trajectories );
	SetComDirection( &trajectories );
	TrajectoriesWriteFile( &trajectories );


	//Inverse Kinematics Calculation
	Configurations configurations;

	SetAngleTrunkYaw( &configurations );
	SetAngleWaist( &configurations );
	LowerBodyInvKin( &configurations, &trajectories );



	system("pause");
}