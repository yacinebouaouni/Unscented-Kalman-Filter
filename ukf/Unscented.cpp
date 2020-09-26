#include "unscented.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
    Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}



void UKF::GenerateSigmaPoints(const VectorXd& x,const MatrixXd& P,MatrixXd* Xsig_out) {

    // set state dimension
    int n_x = x.size();

    // define spreading parameter
    double lambda = 3 - n_x;


    // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

    // Use the method L*LT to compute the square root of P 
    MatrixXd A = P.llt().matrixL();
    
    Xsig.col(0) = x;
    
    for (int i = 1; i < n_x + 1; i++) {

        Xsig.col(i) = x + sqrt(lambda + n_x) * A.col(i-1);
        Xsig.col(n_x+i) = x + sqrt(lambda + n_x) * A.col(i-1);
    }

      // write result
    * Xsig_out = Xsig;
}

