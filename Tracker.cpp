#include<iostream>
#include<Eigen/Dense>
#include"Tracker.h"



Tracker::Tracker(){

	//Set the Tracker at initial position :
	is_initialized_ = false;
	previous_timestamp_ = 0;

	//Initialize process Noise :
	noise_ax = 5.0;
	noise_ay = 5.0;

	//Instantiate Matrices F / Q / H / R and state vector X:

	kf_.x_ = Eigen::VectorXd(4);
	
	kf_.F_ = Eigen::MatrixXd(4, 4);
	kf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	kf_.H_ = Eigen::MatrixXd(2,4);
	kf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	kf_.P_ = MatrixXd(4, 4);
	kf_.P_ << 1, 0, 0, 0,
		      0, 1, 0, 0,
		      0, 0, 1000, 0,
		      0, 0, 0, 1000;

	kf_.R_ = MatrixXd(2, 2);
	kf_.R_ << 0.0225, 0,
		      0, 0.0225;

}

Tracker::~Tracker() {

	std::cout << "End of Tracking ..." << std::endl;
}

void Tracker::ProcessMeasurement(const Measurement& measurement_pack)noexcept {

	/*------------------------------------------------------------------------------------
				If Kalman Filter is not initialized -> First Step (Initialize)
							Else Make a Predict-Update Step 
	--------------------------------------------------------------------------------------*/


	if (!is_initialized_) {

		
		kf_.x_ << measurement_pack.raw_measurements_[0],
				  measurement_pack.raw_measurements_[1],
									0,
									0;

	
		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;

		return;
	}

	/*--------------------------------------------------------------------------------------
					1.Get the Time step dt 
					2.Instantiate Matrix F (State transition Matrix) 
					3.Instantiate Matrix Q (Process noise covariance )
					4.Predict Step .
					5.Update Step.

					F=|1 0 dt 0 |
					  |0 1 0  dt|
					  |0 0 1  0 |
					  |0 0 0  1 |
	---------------------------------------------------------------------------------------*/
	
	double dt = ((double)measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
	
	double dt_2 = dt * dt;
	double dt_3 = dt_2* dt;
	double dt_4 = dt_3*dt;
	
	kf_.F_(0, 2) = dt;
	kf_.F_(1, 3) = dt;

	kf_.Q_ = MatrixXd(4, 4);
	kf_.Q_ <<	dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
				0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
				dt_3 / 2 * noise_ax, 0, dt_2* noise_ax, 0,
				0, dt_3 / 2 * noise_ay, 0, dt_2* noise_ay;


	//Predict Step :

	kf_.Predict();

	// Update Step :

	kf_.Update(measurement_pack.raw_measurements_);



	

}