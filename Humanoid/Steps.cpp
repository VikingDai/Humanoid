#include "Steps.h"

Step::Step( int foot, double theta, Vector2d& position )
{
	foot_ = foot;
	theta_ = theta;
	position_ = position;
	cp_init_.setZero();
	cp_end_.setZero();
	zmp_des_.setZero();
}

string SetFootsteps( Steps *steps )
{
	string err;
	int	method;
	
	cout << "Please insert METHOD number: ";
	cin >> method;
	switch(method){
		case 0:
			cout << "Creating one testing footstep..." << endl;
			steps->push_back( new Step( 1, pi/2, Vector2d(0, 0)) );
			break;

		case 1:
			cout << "Creating simple straight 10 footsteps..." << endl;
			steps->resize(10);
			(*steps)[0] = new Step( 1, 0, Vector2d(0, 0));
     		(*steps)[1] = new Step( 0, 0, Vector2d(0, 20));
     		(*steps)[2] = new Step( 1, 0, Vector2d(25, 0));
     		(*steps)[3] = new Step( 0, 0, Vector2d(50, 20));
     		(*steps)[4] = new Step( 1, 0, Vector2d(75, 0));
     		(*steps)[5] = new Step( 0, 0, Vector2d(100, 20));
     		(*steps)[6] = new Step( 1, 0, Vector2d(125, 0)); 
     		(*steps)[7] = new Step( 0, 0, Vector2d(150, 20));
			(*steps)[8] = new Step( 1, 0, Vector2d(175,0));
			(*steps)[9] = new Step( 0, 0, Vector2d(175, 20));
			break;

		default:
			method = -1;
			break;

	}


	// stepping legal test
	if(method == -1)
		err = "Wrong method input.";
	else if(steps->size() < 2 )
		err = "Size of steps < 2";
	// TODO
	// foot step legal test
	// if not legal, set error message to err
	// examples as above
	else 
		err = "SUCCESS";

	return err;

}
string SetAllCP( Steps *steps )
{
	cout << "Calculating CPs of all footsteps..." << endl;

	string err;
	//num_steps = steps.size();
	int num_steps = steps->size();
	int PreviewSteps = MaxPreviewSteps;

	if( (num_steps-1) < PreviewSteps )		// If there's not enough step to estimate the cp_ref,
		PreviewSteps = num_steps-1;			// decrease the preview step number to the step number

	for(int i = 0; i < (num_steps - PreviewSteps); i++){
		// updating s(i+n-1).cp_end
		if(i == num_steps - PreviewSteps -1)     // if this is not the last round to updatie the CPs
			(*steps)[num_steps-2]->cp_end_ = ((*steps)[num_steps-2]->position_ + (*steps)[num_steps-1]->position_) / 2;  // second last footprint CP = mid point of last two footstep center
		else
			(*steps)[i + PreviewSteps - 1]->cp_end_ = (*steps)[i + PreviewSteps]->position_;			

		// back tracking the s(i+n-1).cp_init (= s(i-n-2).cp_end)
		for(int j = (i + PreviewSteps - 1); j>=i; j--){
			if(j == 0)
				(*steps)[j]->cp_init_ = ((*steps)[j]->position_ + (*steps)[1]->position_) / 2;
			else{
				Vector2d temp_vec = (*steps)[j]->cp_end_ - (*steps)[j]->position_;
				(*steps)[j]->cp_init_ = (*steps)[j]->position_ + CP_Offset * temp_vec.normalized();
				(*steps)[j-1]->cp_end_ = (*steps)[j]->cp_init_;
			}
		}

	}

	// capture point legal test
	if(0){
	// TODO
	// check if the CP generated is legal
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;

}
string SetAllZMP( Steps *steps)
{
	cout << "Calculating ZMPs of all footsteps..." << endl;

	string err;
	int num_steps = steps->size();


	for(int i=0; i<num_steps; i++){
		if(i == num_steps-1) //if last step
			(*steps)[i]->zmp_des_ = ((*steps)[i-1]->position_ + (*steps)[i]->position_) / 2;
		else{
			double b = exp(Omega * StepTime);
			(*steps)[i]->zmp_des_ = 1/(1-b) * (*steps)[i]->cp_end_ - b/(1-b) * (*steps)[i]->cp_init_;
		}
	}


	// ZMP legal test
	if(0){
	// TODO
	// check if the ZMP generated is legal
	// for example, zmp should be inside the polygon
	// if not legal, set error message to err
	}
	else 
		err = "SUCCESS";

	return err;

}
void PrintAllSteps( const Steps *steps )
{
	for(unsigned int i=0; i<steps->size(); i++){
		cout << i << " step:" << endl;
		cout << (*steps)[i]->foot_ << endl;
		cout << (*steps)[i]->theta_ << endl;
		cout << (*steps)[i]->position_.transpose() << endl; 
		cout << (*steps)[i]->cp_init_.transpose() << endl;
		cout << (*steps)[i]->cp_end_.transpose() << endl;
		cout << (*steps)[i]->zmp_des_.transpose() << endl;
		cout << endl;
	}

}