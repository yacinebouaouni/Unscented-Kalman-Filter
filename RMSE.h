#pragma once
#include<iostream>
#include<Eigen/Dense>
#include<vector>

using Eigen::VectorXd;
using std::vector;


VectorXd RMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& truth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	/*--------------------------------------------------------------------------------------------
							1. Check if the estimations vector is not empty 
							2. Check if we have the same number of ground truth and estimates
	---------------------------------------------------------------------------------------------*/
	
	if (estimations.size()==0) {

		std::cout << "Estimation vector is empty !" << std::endl;
		return rmse;
	}

	if (!(estimations.size() == truth.size())) {

		std::cout << " Estimations and ground truth size mismatch !" << std::endl;
		return rmse;
	}


	/*----------------------------------------------------------------------------------------------
										Compute RMSE Score 
	-----------------------------------------------------------------------------------------------*/
	
	for (int i = 0; i < estimations.size(); i++) {

		VectorXd residual = estimations[i] - truth[i];
		residual = residual.array() * residual.array();

		rmse += residual;
	}

	//Compute the mean : 
	rmse = rmse / (estimations.size());

	//Compute the squared root:
	rmse = rmse.array().sqrt();

	return rmse;

}