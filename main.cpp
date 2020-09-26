#include<iostream>
#include<fstream>
#include<Eigen/Dense>
#include<vector>
#include"kalman.h"
#include"measurement.h"
#include"Tracker.h"


using namespace Eigen;
using namespace std;


int main3() {

	
	string file_name_in = "./data/data.txt";
	ifstream instream;
	instream.open(file_name_in, ios::in);

	//Check if the file stream is open
	if (!instream.is_open()) {
		cout << "File Stream Error : Cannot open File " << file_name_in << endl;
	}

	

	//i -> number of measurements 
	short int num_measurements=3;
	int i = 0;
	string raw;

	std::vector<Measurement> measurements;
	while (getline(instream, raw) && (i <= num_measurements)) {

		Measurement measurepkg;
		stringstream raw_stream(raw);

		/*------------------------------------------------------
					First string is a sensor type 
					Check if it's a Lidar or Radar

					*---------------Lidar------------* 
					Second string -> x
					Third string  -> y
					4 th   string ->timestamp
		 ------------------------------------------------------- */
		
		string sensor_type;
		int64_t timestamp;
		raw_stream >> sensor_type;


		if (sensor_type.compare("L")==0) {
			measurepkg.sensor_type_ = Measurement::LIDAR;
			double x, y;
			
			raw_stream >> x;
			raw_stream >> y;
			raw_stream >> timestamp;
			measurepkg.raw_measurements_ << x, y;
			measurepkg.timestamp_ = timestamp;

			measurements.push_back(measurepkg);

			i++;
		}
		else if(sensor_type.compare("R") == 0) {
			measurepkg.sensor_type_ = Measurement::RADAR;
			continue;
		}

	}


	size_t N = measurements.size();
	Tracker* tracker=new Tracker();

	for (Measurement m : measurements) {

		tracker->ProcessMeasurement(m);

	}

	//Close the stream :
	if (instream.is_open()) {
		instream.close();
	}

	delete tracker;
	return 0;
}