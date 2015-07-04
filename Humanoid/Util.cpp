#include "Util.h"

double round(double number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

void EigenWriteFile( VectorXd& vector, string file, string path )
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
void EigenWriteFile( MatrixXd& matrix, string file, string path )
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