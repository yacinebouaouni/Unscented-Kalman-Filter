
#include"kalman.h"
#include<iostream>
#include<Eigen/Dense>

KalmanFilter::KalmanFilter() {

}

KalmanFilter::~KalmanFilter() {

}


void KalmanFilter::Predict() {

	//Predict the new state 
	this->x_ = this->x_ * this->F_;

	//Predict the new covariance:
	this->P_ = this->F_ * this->P_ * (this->F_).transpose() + this->Q_;

}


void KalmanFilter::Update(const Eigen::VectorXd &z) {

	Eigen::VectorXd Inov = z - this->H_ * this->x_;
	Eigen::MatrixXd Inov_cov = this->H_ * this->P_ * (this->H_).transpose() + this->R_;
	Eigen::MatrixXd Kalman_Gain = this->P_ * (this->H_).transpose() * (Inov_cov).inverse();




}