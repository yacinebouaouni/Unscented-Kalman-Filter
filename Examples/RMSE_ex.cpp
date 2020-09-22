#include"../RMSE.h"
#include<Eigen/Dense>
#include<iostream>
#include<fstream>

int main() {

	std::string file_name = "./data/RMSE.txt";
	std::fstream file_stream;
	file_stream.open(file_name,std::ios::in);

	if (!file_stream.is_open()) {

		std::cout << "File Not Found !" << std::endl;
		return 0;
	}

	std::string line;
	int i = 0;

	std::vector<VectorXd> truth;
	std::vector<VectorXd> predicted;

	while (std::getline(file_stream, line)) {

		std::stringstream sstream(line);
		double px, py, vx, vy;

		sstream >> px;
		sstream >> py;
		sstream >> vx;
		sstream >> vy;

		VectorXd vector(4);
		vector << px, py, vx, vy;

		if (i % 2 == 0) {
			truth.push_back(vector);
		}

		else predicted.push_back(vector);

		i++;

	}

	std::cout << "size of predictions " << predicted.size() << std::endl;
	std::cout << "size of groundtruth " << truth.size() << std::endl;
	VectorXd rmse = RMSE(predicted, truth);

	std::cout << "RMSE = " << std::endl << rmse;

	return 0;

}