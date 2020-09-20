#pragma once
#ifndef KALMAN_H_
#define KALMAN_H_
#include<Eigen/Dense>

using namespace Eigen;

class KalmanFilter {

	public:

		KalmanFilter();
		virtual ~KalmanFilter();

		//Predict step 
		void Predict();
		//Update step : z -> Measurement 
		void Update(const VectorXd& z);

		/*-----------------------------------------
				Kalman Filter Matrices 
		-------------------------------------------*/
		
		VectorXd x_; // state vector
		MatrixXd P_; // state covariance matrix
		MatrixXd F_; // state transistion matrix
		MatrixXd Q_; // process covariance matrix
		MatrixXd H_; // measurement matrix
		MatrixXd R_; // measurement covariance matrix
	};

#endif  // KALMAN_FILTER_H_