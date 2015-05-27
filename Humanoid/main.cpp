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
	SetAllCP( &steps );
	SetAllZMP( &steps );

	//Trajectory Calculation
	Trajectories trajectories( &steps );
	
	SetDiscreteZMP( &trajectories, &steps );
	SetFootTrajectory( &trajectories, &steps );
	SetDiscreteCOM( &trajectories );
	TrajectoriesWriteFile( &trajectories );

	//Inverse Kinematics Calculation
	Configurations configurations;

	


	system("pause");
}