#include "Trajectories.h"


Trajectories::Trajectories( Steps *steps )
{
	int num_steps = steps->size();
	TotalTimeFrame = PrepareTimeFrame + DoublePhaseTimeFrame + (SinglePhaseTimeFrame + DoublePhaseTimeFrame) * (num_steps - 2) + HoldTimeFrame;

	zmp.resize(3, TotalTimeFrame);
	com.resize(3, TotalTimeFrame);
	left_foot.resize(3, TotalTimeFrame);
	right_foot.resize(3, TotalTimeFrame);

	com_direction.resize(TotalTimeFrame);
	left_foot_direction.resize(TotalTimeFrame);
	right_foot_direction.resize(TotalTimeFrame);

	zmp.setZero();
	com.setZero();
	left_foot.setZero();
	right_foot.setZero();

	GetSplineVec(&zmp_spline_vec, DoublePhaseTimeFrame, ZmpSplineType);
	GetSplineVec(&foot_spline_vec, SinglePhaseTimeFrame, FootTrajectorySplineType);
	GetSwingVec(&foot_swing_vec, SinglePhaseTimeFrame);
}

string SetZmpTrajectory( Trajectories *trajectories, Steps *steps )
{
	string err;
	cout << "Calculating Zmp trajectory..." << endl;

	int time_flag = PrepareTimeFrame;
	int num_steps = steps->size();

	VectorXd single_ones_vec = VectorXd(SinglePhaseTimeFrame).setOnes();
	VectorXd double_ones_vec = VectorXd(DoublePhaseTimeFrame).setOnes();

	// Supposed the robot is walking on flat plane!
	trajectories->zmp.row(2).setZero();

	// ex. 6 footstep = 4 steps = (d+s+d) + (s+d) + (s+d) + (s+d) = d + (s+d)*4
	for(int i=0; i<num_steps-2; i++){
		// first step double phase
		if(i==0){
			trajectories->zmp.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[0]->zmp_des_(0) * double_ones_vec
																			+ ((*steps)[1]->zmp_des_(0) - (*steps)[0]->zmp_des_(0)) * trajectories->zmp_spline_vec;
			trajectories->zmp.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[0]->zmp_des_(1) * double_ones_vec
																			+ ((*steps)[1]->zmp_des_(1) - (*steps)[0]->zmp_des_(1)) * trajectories->zmp_spline_vec;
			time_flag += DoublePhaseTimeFrame;
		}

		// single phase in every step
		trajectories->zmp.row(0).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i+1]->zmp_des_(0) * single_ones_vec;
		trajectories->zmp.row(1).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i+1]->zmp_des_(1) * single_ones_vec;
		time_flag += SinglePhaseTimeFrame;
    
		// double phase in every step
		trajectories->zmp.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+1]->zmp_des_(0) * double_ones_vec
																		+ ((*steps)[i+2]->zmp_des_(0) - (*steps)[i+1]->zmp_des_(0)) * trajectories->zmp_spline_vec;
		trajectories->zmp.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+1]->zmp_des_(1) * double_ones_vec
																		+ ((*steps)[i+2]->zmp_des_(1) - (*steps)[i+1]->zmp_des_(1)) * trajectories->zmp_spline_vec;
		time_flag += DoublePhaseTimeFrame;
	}

	trajectories->zmp.row(0).head(PrepareTimeFrame).setConstant(trajectories->zmp(0, PrepareTimeFrame));
	trajectories->zmp.row(1).head(PrepareTimeFrame).setConstant(trajectories->zmp(1, PrepareTimeFrame));

	trajectories->zmp.row(0).tail(HoldTimeFrame).setConstant(trajectories->zmp(0, TotalTimeFrame - HoldTimeFrame -1));
	trajectories->zmp.row(1).tail(HoldTimeFrame).setConstant(trajectories->zmp(1, TotalTimeFrame - HoldTimeFrame -1));


	// zmp trajectory legal test
	if(0){
	// TODO
	// check if the zmp trajectory generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
string SetFootTrajectory( Trajectories *trajectories, Steps *steps )
{
	string err;
	cout << "Calculating foot trajectory..." << endl;

	int num_steps = steps->size();
	int time_flag = PrepareTimeFrame;

	
	VectorXd single_ones_vec = VectorXd(SinglePhaseTimeFrame).setOnes();
	VectorXd double_ones_vec = VectorXd(DoublePhaseTimeFrame).setOnes();


	// ex. 6 footstep = 4 steps = (d+s+d) + (s+d) + (s+d) + (s+d) = d + (s+d)*4
	// supposed we swing the right foot first, and if it's not true, switch it back in the end
	for(int i=0; i<num_steps-2; i++){
		// first double phase move
		if(i==0){
			trajectories->right_foot.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[0]->position_(0) * double_ones_vec;
			trajectories->right_foot.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[0]->position_(1) * double_ones_vec;
			trajectories->right_foot.row(2).segment(time_flag,DoublePhaseTimeFrame).setZero();
			
			trajectories->left_foot.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[1]->position_(0) * double_ones_vec;
			trajectories->left_foot.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[1]->position_(1) * double_ones_vec;
			trajectories->left_foot.row(2).segment(time_flag,DoublePhaseTimeFrame).setZero();
			
			time_flag += DoublePhaseTimeFrame;
		}

		
		if(i%2 == 0) {	// if swinging the right foot in this step
			// single phase in every step
			trajectories->right_foot.row(0).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i]->position_(0) * single_ones_vec + ((*steps)[i+2]->position_(0) - (*steps)[i]->position_(0)) * trajectories->foot_spline_vec;
			trajectories->right_foot.row(1).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i]->position_(1) * single_ones_vec + ((*steps)[i+2]->position_(1) - (*steps)[i]->position_(1)) * trajectories->foot_spline_vec;
			trajectories->right_foot.row(2).segment(time_flag,SinglePhaseTimeFrame) = SwingLegHeight * trajectories->foot_swing_vec;

			trajectories->left_foot.row(0).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i+1]->position_(0) * single_ones_vec;
			trajectories->left_foot.row(1).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i+1]->position_(1) * single_ones_vec;
			trajectories->left_foot.row(2).segment(time_flag,SinglePhaseTimeFrame).setZero();

			time_flag += SinglePhaseTimeFrame;

			// double phase in every step
			trajectories->right_foot.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+2]->position_(0) * double_ones_vec;
			trajectories->right_foot.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+2]->position_(1) * double_ones_vec;
			trajectories->right_foot.row(2).segment(time_flag,DoublePhaseTimeFrame).setZero();
	
			trajectories->left_foot.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+1]->position_(0) * double_ones_vec;
			trajectories->left_foot.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+1]->position_(1) * double_ones_vec;
			trajectories->left_foot.row(2).segment(time_flag,DoublePhaseTimeFrame).setZero();

			time_flag += DoublePhaseTimeFrame;
		} 
		else {	// if swinging the left foot in this step
			// single phase in every step
			trajectories->right_foot.row(0).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i+1]->position_(0) * single_ones_vec;
			trajectories->right_foot.row(1).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i+1]->position_(1) * single_ones_vec;
			trajectories->right_foot.row(2).segment(time_flag,SinglePhaseTimeFrame).setZero();

			trajectories->left_foot.row(0).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i]->position_(0) * single_ones_vec + ((*steps)[i+2]->position_(0) - (*steps)[i]->position_(0)) * trajectories->foot_spline_vec;
			trajectories->left_foot.row(1).segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i]->position_(1) * single_ones_vec + ((*steps)[i+2]->position_(1) - (*steps)[i]->position_(1)) * trajectories->foot_spline_vec;
			trajectories->left_foot.row(2).segment(time_flag,SinglePhaseTimeFrame) = SwingLegHeight * trajectories->foot_swing_vec;

			time_flag += SinglePhaseTimeFrame;

			// double phase in every step
			trajectories->right_foot.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+1]->position_(0) * double_ones_vec;
			trajectories->right_foot.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+1]->position_(1) * double_ones_vec;
			trajectories->right_foot.row(2).segment(time_flag,DoublePhaseTimeFrame).setZero();
	
			trajectories->left_foot.row(0).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+2]->position_(0) * double_ones_vec;
			trajectories->left_foot.row(1).segment(time_flag,DoublePhaseTimeFrame) = (*steps)[i+2]->position_(1) * double_ones_vec;
			trajectories->left_foot.row(2).segment(time_flag,DoublePhaseTimeFrame).setZero();

			time_flag += DoublePhaseTimeFrame;
		}
	}

	trajectories->right_foot.row(0).head(PrepareTimeFrame).setConstant(trajectories->right_foot(0, PrepareTimeFrame));
	trajectories->right_foot.row(1).head(PrepareTimeFrame).setConstant(trajectories->right_foot(1, PrepareTimeFrame));
	trajectories->right_foot.row(2).head(PrepareTimeFrame).setConstant(trajectories->right_foot(2, PrepareTimeFrame));

	trajectories->left_foot.row(0).head(PrepareTimeFrame).setConstant(trajectories->left_foot(0, PrepareTimeFrame));
	trajectories->left_foot.row(1).head(PrepareTimeFrame).setConstant(trajectories->left_foot(1, PrepareTimeFrame));
	trajectories->left_foot.row(2).head(PrepareTimeFrame).setConstant(trajectories->left_foot(2, PrepareTimeFrame));

	trajectories->right_foot.row(0).tail(HoldTimeFrame).setConstant(trajectories->right_foot(0, TotalTimeFrame - HoldTimeFrame -1));
	trajectories->right_foot.row(1).tail(HoldTimeFrame).setConstant(trajectories->right_foot(1, TotalTimeFrame - HoldTimeFrame -1));
	trajectories->right_foot.row(2).tail(HoldTimeFrame).setConstant(trajectories->right_foot(2, TotalTimeFrame - HoldTimeFrame -1));

	trajectories->left_foot.row(0).tail(HoldTimeFrame).setConstant(trajectories->left_foot(0, TotalTimeFrame - HoldTimeFrame -1));
	trajectories->left_foot.row(1).tail(HoldTimeFrame).setConstant(trajectories->left_foot(1, TotalTimeFrame - HoldTimeFrame -1));
	trajectories->left_foot.row(2).tail(HoldTimeFrame).setConstant(trajectories->left_foot(2, TotalTimeFrame - HoldTimeFrame -1));



	////////////////////////////if(strcmp((*steps)[0]->foot_,"right")) { // if right foot is the first swing leg/////////////////////////////////////////////////////////////////////////////////////

	


	// foot trajectory legal test
	if(0){
	// TODO
	// check if the foot trajectory generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
string SetFootDirection( Trajectories *trajectories, Steps *steps )
{
	string err;
	cout << "Calculating foot direction..." << endl;

	int num_steps = steps->size();
	int time_flag = PrepareTimeFrame;

	
	VectorXd single_ones_vec = VectorXd(SinglePhaseTimeFrame).setOnes();
	VectorXd double_ones_vec = VectorXd(DoublePhaseTimeFrame).setOnes();


	// ex. 6 footstep = 4 steps = (d+s+d) + (s+d) + (s+d) + (s+d) = d + (s+d)*4
	// supposed we swing the right foot first, and if it's not true, switch it back in the end
	for(int i=0; i<num_steps-2; i++){
		// first double phase move
		if(i==0){
			trajectories->right_foot_direction.segment(time_flag,DoublePhaseTimeFrame).setConstant((*steps)[0]->direction_);
			trajectories->left_foot_direction.segment(time_flag,DoublePhaseTimeFrame).setConstant((*steps)[1]->direction_);
			
			time_flag += DoublePhaseTimeFrame;
		}

		
		if(i%2 == 0) {	// if swinging the right foot in this step
			// single phase in every step
			trajectories->right_foot_direction.segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i]->direction_ * single_ones_vec + ((*steps)[i+2]->direction_ - (*steps)[i]->direction_) * trajectories->foot_spline_vec;
			trajectories->left_foot_direction.segment(time_flag,SinglePhaseTimeFrame).setConstant((*steps)[i+1]->direction_);


			time_flag += SinglePhaseTimeFrame;

			// double phase in every step
			trajectories->right_foot_direction.segment(time_flag,SinglePhaseTimeFrame).setConstant((*steps)[i+2]->direction_);
			trajectories->left_foot_direction.segment(time_flag,SinglePhaseTimeFrame).setConstant((*steps)[i+1]->direction_);

			time_flag += DoublePhaseTimeFrame;
		} 
		else {	// if swinging the left foot in this step
			// single phase in every step
			trajectories->right_foot_direction.segment(time_flag,SinglePhaseTimeFrame).setConstant((*steps)[i+1]->direction_);
			trajectories->left_foot_direction.segment(time_flag,SinglePhaseTimeFrame) = (*steps)[i]->direction_ * single_ones_vec + ((*steps)[i+2]->direction_ - (*steps)[i]->direction_) * trajectories->foot_spline_vec;



			time_flag += SinglePhaseTimeFrame;

			// double phase in every step
			trajectories->right_foot_direction.segment(time_flag,SinglePhaseTimeFrame).setConstant((*steps)[i+1]->direction_);
			trajectories->left_foot_direction.segment(time_flag,SinglePhaseTimeFrame).setConstant((*steps)[i+2]->direction_);

			time_flag += DoublePhaseTimeFrame;
		}
	}

	trajectories->right_foot_direction.head(PrepareTimeFrame).setConstant(trajectories->right_foot_direction(PrepareTimeFrame));
	trajectories->left_foot_direction.head(PrepareTimeFrame).setConstant(trajectories->left_foot_direction(PrepareTimeFrame));

	trajectories->right_foot_direction.tail(HoldTimeFrame).setConstant(trajectories->right_foot_direction(TotalTimeFrame - HoldTimeFrame -1));
	trajectories->left_foot_direction.tail(HoldTimeFrame).setConstant(trajectories->left_foot_direction(TotalTimeFrame - HoldTimeFrame -1));


	////////////////////////////if(strcmp((*steps)[0]->foot_,"right")) { // if right foot is the first swing leg/////////////////////////////////////////////////////////////////////////////////////

	


	// foot direction legal test
	if(0){
	// TODO
	// check if the foot direction generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
string SetComTrajectory( Trajectories *trajectories )
{
	string err;
	cout << "Calculating Com trajectory..." << endl;

	// temp acceptable solution for com 
	trajectories->com.row(0) = 0.5 * (trajectories->left_foot.row(0) + trajectories->right_foot.row(0));
	trajectories->com.row(1) = 0.5 * (trajectories->left_foot.row(1) + trajectories->right_foot.row(1));
	trajectories->com.row(2).setConstant(ComHeight);

	// com trajectory legal test
	if(0){
	// TODO
	// check if the com trajectory generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
string SetComDirection( Trajectories *trajectories )
{
	string err;
	cout << "Calculating Com direction..." << endl;
	

	trajectories->com_direction = 0.5 * (trajectories->left_foot_direction + trajectories->right_foot_direction);


	// Com direction legal test
	if(0){
	// TODO
	// check if the Com direction generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;
}
void GetSplineVec(VectorXd *spline_vec, int vec_length, int spline_type)
{
	spline_vec->resize(vec_length);

	if(spline_type == 1){
		spline_vec->row(0).setLinSpaced(vec_length, 0, 1-1/vec_length);
	}
	else if(spline_type == 3){
		double	x1 = 0,
				x2 = vec_length,
				y1 = 0,
				ydot1 = 0,
				y2 = 1,
				ydot2 = 0;

		MatrixXd A(4,4);
		A << pow(x1,3.0),		pow(x1,2.0),	x1,		1,
			 3*pow(x1,2.0),		2*x1,			1,		0,
			 pow(x2,3.0),		pow(x2,2.0),	x2,		1,
			 3*pow(x2,2.0),		2*x2,			1,		0;

		
		VectorXd B(4);
		B << y1, ydot1, y2, ydot2;

		VectorXd S(4);
		S = A.inverse() * B;

		for(double i=0; i<vec_length; i++){
			(*spline_vec)((int)i) = S(0) * pow(i,3) + S(1) * pow(i,2) + S(2) * i + S(3);		//warning C4244: '引數' : 將 'double' 轉換為 '__w64 int'，由於型別不同，可能導致資料遺失

		}

	}
	else if(spline_type == 5){
		double	x1 = 0,
				x2 = vec_length,
				y1 = 0,
				y1Vel = 0,
				y1Acc = 0,
				y2 = 1,
				y2Vel = 0,
				y2Acc = 0;
			
		MatrixXd A(6,6);
		A << pow(x1,5),		pow(x1,4),		pow(x1,3),		pow(x1,2),	pow(x1,1),	1,
			 5*pow(x1,4),	4*pow(x1,3),	3*pow(x1,2),	2*x1,		1,			0,
			 20*pow(x1,3),	12*pow(x1,2),	6*x1,			2,			0,			0,
			 pow(x2,5),		pow(x2,4),		pow(x2,3),		pow(x2,2),	pow(x2,1),	1,
			 5*pow(x2,4),	4*pow(x2,3),	3*pow(x2,2),	2*x2,		1,			0,
			 20*pow(x2,3),	12*pow(x2,2),	6*x2,			2,			0,			0;

		VectorXd B(6);
		B << y1, y1Vel, y1Acc, y2, y2Vel, y2Acc;

		VectorXd S(6);
		S = A.inverse() * B;

		for(double i=0; i<vec_length; i++){
			(*spline_vec)((int)i) = S(0) * pow(i,5) + S(1) * pow(i,4) + S(2) * pow(i,3) + S(3) * pow(i,2) + S(4) * i + S(5);		//warning C4244: '引數' : 將 'double' 轉換為 '__w64 int'，由於型別不同，可能導致資料遺失
		}

	}
}
void GetSwingVec(VectorXd *swing_vec, int vec_length)
{
	swing_vec->resize(vec_length);

	// This is only one of a simple swing leg curve
	// can test another kind of curves by changing the code below
	for(int i=0; i<vec_length; i++){
		(*swing_vec)(i) = -1 * pow(-(2.0/vec_length), 4) * pow((i-vec_length/2.0), 4) + 1;
	}
}
void TrajectoriesWriteFile( Trajectories *trajectories )
{
	
	EigenWriteFile(trajectories->zmp, "cpp_zmp", WriteFilePath);
	EigenWriteFile(trajectories->com, "cpp_com", WriteFilePath);
	EigenWriteFile(trajectories->left_foot, "cpp_left_foot", WriteFilePath);
	EigenWriteFile(trajectories->right_foot, "cpp_right_foot", WriteFilePath);

	EigenWriteFile(trajectories->com_direction, "cpp_com_direction", WriteFilePath);
	EigenWriteFile(trajectories->left_foot_direction, "cpp_left_foot_direction", WriteFilePath);
	EigenWriteFile(trajectories->right_foot_direction, "cpp_right_foot_direction", WriteFilePath);

	EigenWriteFile(trajectories->zmp_spline_vec, "cpp_zmp_spline_vec", WriteFilePath);
	EigenWriteFile(trajectories->foot_spline_vec, "cpp_foot_spline_vec", WriteFilePath);
	EigenWriteFile(trajectories->foot_swing_vec, "cpp_foot_swing_vec", WriteFilePath);

}
void EigenWriteFile( VectorXd vector, string file, string path )
{
	string file_name = path + file;
	ofstream myfile(file_name);
	if(!myfile.is_open()) {
		cout << "Fail to open \"" << file_name << "\"" << endl;
	}
	else {
		myfile << vector;	
	}
	myfile.close();
}
void EigenWriteFile( MatrixXd matrix, string file, string path )
{
	string file_name = path + file;
	ofstream myfile(file_name);
	if(!myfile.is_open()) {
		cout << "Fail to open \"" << file_name << "\"" << endl;
	}
	else {
		myfile << matrix;	
	}
	myfile.close();
}