#include<iostream>
#include<fstream>
#include<Eigen/Dense>
#include"kalman.h"
#include"measurement_package.h"


using namespace Eigen;
using namespace std;


int main() {

	
	string file_name_in = "./data/data.txt";
	ifstream instream;
	instream.open(file_name_in, ios::in);

	//Check if the file stream is open
	if (!instream.is_open()) {
		cout << "File Stream Error : Cannot open File " << file_name_in << endl;
	}

	//

	//i -> number of measurements 
	short int num_measurements=20;
	int i = 0;
	string raw;

	while (getline(instream, raw) && (i <= num_measurements)) {

		
		stringstream raw_stream(raw);

		//First string is a sensor type:
		

	}
}