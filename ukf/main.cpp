#include <iostream>
#include<Eigen/Dense>
#include "unscented.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

int main() {

	// Create a UKF instance
	UKF ukf;

	int n_x(5);

	// set example state
	VectorXd x(n_x);
	x << 5.7441,
		1.3800,
		2.2049,
		0.5015,
		0.3528;

	// set example covariance matrix
	MatrixXd P = MatrixXd(n_x, n_x);
	P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
		-0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
		0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
		-0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
		-0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

	//Set noise mean to 0 :
	VectorXd noise = VectorXd::Zero(2);

	//Set noise covariance matrix to sigma_a^2 and sigma_yaw^2
	double sigma_a(0.2*0.2), sigma_yaw(0.2*0.2);

	MatrixXd Q(2, 2);
	Q << sigma_a , 0,
		 0,sigma_yaw;



	MatrixXd Xsig = MatrixXd(5, 11);
	ukf.GenerateSigmaPoints(x,P,&Xsig);

	// print result
	//std::cout << "Xsig = " << std::endl << Xsig << std::endl;

	/**
 * expected result:
 * Xsig =
 *  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
 *    1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
 *  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
 *  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
 *  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
 */


	ukf.AugmentedSigmaPoints(x, P, noise, Q,&Xsig);
	// print result
	//std::cout << "Xsig = " << std::endl << Xsig << std::endl;

	/*
		5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
		1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
		2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
		0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
		0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
		0        0        0        0        0        0  0.34641        0        0        0        0        0        0 - 0.34641        0
		0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 - 0.34641
	*/




	MatrixXd Pred_Sig= MatrixXd(5, 15);
	ukf.SigmaPointPrediction(Xsig,&Pred_Sig);

	//cout << "The predictions are : \n" << Pred_Sig <<endl;

	/*
		5.93553   6.0625  5.92217   5.9415  5.92361  5.93516  5.93705  5.93553  5.80833  5.94481  5.92935  5.94553  5.93589  5.93401  5.93553
		1.48939  1.44673  1.66483  1.49719    1.508  1.49001  1.49022  1.48939   1.5308  1.31288  1.48182  1.46967  1.48876  1.48855  1.48939
		2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.23954   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049  2.17026   2.2049
		0.53678 0.473388 0.678099 0.554557 0.643644 0.543372  0.53678 0.538512 0.600172 0.395461 0.519003 0.429916 0.530188  0.53678 0.535048
		 0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528 0.387441 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528 0.318159
	
	*/

	VectorXd x_pred(5);
	MatrixXd P_pred(5, 5);
	ukf.PredictMeanAndCovariance(Pred_Sig, &x_pred, &P_pred);

	//cout << "The mean of Sig points = \n" << x_pred << endl;
	//cout << "The covariance matrix = \n" << P_pred << endl;

	/*
		The mean of Sig points =
		5.93446
		1.48886
		 2.2049
		0.53678
		 0.3528
		The covariance matrix =
		 0.00548035   -0.002499  0.00340508 -0.00357408  -0.0030908
		  -0.002499   0.0110543  0.00151778  0.00990746  0.00806631
		 0.00340508  0.00151778      0.0058     0.00078      0.0008
		-0.00357408  0.00990746     0.00078    0.011924     0.01125
		 -0.0030908  0.00806631      0.0008     0.01125      0.0127
	*/

	VectorXd z_meas(3);
	MatrixXd S(3, 3);
	MatrixXd sig_meas(3,15);
	ukf.PredictRadarMeasurement(Pred_Sig, &z_meas, &S,&sig_meas);
	
	//cout << "The mean z = \n " << z_meas << std::endl;
	//cout << "The covairance matrix S = \n" << S << std::endl;

	/*
		The mean z =
		6.11934
		0.245834
		2.10274
		The covairance matrix S =
		0.0946302 - 0.000145123   0.00408742
		- 0.000145123  0.000624209 - 0.000781362
		0.00408742 - 0.000781362    0.0180473
	*/

	MatrixXd T(5, 3);

	cout << "pred_sig=" << Pred_Sig << endl;
	cout << "x_pred = " << x_pred << endl;
	cout << "sig_meas =" << sig_meas << endl;
	cout << "z_meas = " << z_meas << endl;


	ukf.CrossCorrelationT(Pred_Sig, x_pred, z_meas,sig_meas,&T);

	cout << "Cross correlation Matrix T = \n" << T << endl;


	return 0;
}