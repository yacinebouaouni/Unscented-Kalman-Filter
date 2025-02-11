#include "unscented.h"
#include <iostream>
#include<Cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
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
        Xsig.col(n_x+i) = x - sqrt(lambda + n_x) * A.col(i-1);
    }

      // write result
    * Xsig_out = Xsig;
}


void UKF::AugmentedSigmaPoints(const VectorXd& x, const MatrixXd& P,const VectorXd& noise,const MatrixXd& Q,MatrixXd* Xsig_out) {


        int n_x = x.size() + noise.size();
        VectorXd aug_x = VectorXd(n_x);
        MatrixXd aug_P(n_x, n_x);

        aug_x.head(x.size()) = x;
        aug_x.tail(noise.size()) = noise;

        aug_P.fill(0.0);


        aug_P.topLeftCorner(x.size(), x.size()) = P;
        aug_P.bottomRightCorner(noise.size(), noise.size()) = Q;



    // define spreading parameter
    double lambda = 3 - n_x;

    // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

    // Use the method L*LT to compute the square root of P 
    MatrixXd A = aug_P.llt().matrixL();

    Xsig.col(0) = aug_x;

    for (int i = 1; i < n_x + 1; i++) {

        Xsig.col(i) = aug_x + sqrt(lambda + n_x) * A.col(i-1);
        Xsig.col(n_x + i) = aug_x - sqrt(lambda + n_x) * A.col(i-1);
    }

    // write result
    *Xsig_out = Xsig;


}

void UKF::SigmaPointPrediction(const MatrixXd& SigPoints,MatrixXd* Xsig_out) {

    
    double dt = 0.1;
    int size_x = 5;
    int size_aug = 7;
    MatrixXd Preds_Sig(size_x, 2 * size_aug + 1);;
    
    for (int i = 0; i < 2 * size_aug + 1; i++) {

        /*----------------------------------------------------------------
                  Extract the state vector for the Sigma point i
        ------------------------------------------------------------------*/
        double p_x = SigPoints(0, i);
        double p_y = SigPoints(1, i);
        double v = SigPoints(2, i);
        double yaw = SigPoints(3, i);
        double yaw_r = SigPoints(4, i);
        double noise_a = SigPoints(5, i);
        double noise_y = SigPoints(6, i);

        /*-------------------------------------------------------------------
                    Add change rate of state to  the old state Xk 
        ----------------------------------------------------------------------*/
        double p_x_p, p_y_p, v_p, yaw_p, yaw_r_p;

        v_p = v ;
        yaw_p = yaw + yaw_r * dt;
        yaw_r_p = yaw_r;
       
        //Avoid dividing By 0 if yaw_r=0:

        if (fabs(yaw_r)<0.001) {

            p_x_p = p_x + v * cos(yaw) * dt;
            p_y_p = p_y + v * sin(yaw) * dt;

        }

        else {
            
            p_x_p = p_x + (v / yaw_r) * (sin(yaw_p) - sin(yaw));
            p_y_p = p_y + (v / yaw_r) * (-cos(yaw_p) + cos(yaw));

        }

        /*---------------------------------------------------------------------
                    Add the noise contribution to the state vector 
        -------------------------------------------------------------------------*/
        
        p_x_p += 0.5 * noise_a * dt * dt * cos(yaw);
        p_y_p += 0.5 * noise_a * dt * dt * sin(yaw);
        v_p += noise_a * dt;
        yaw_p += 0.5 * noise_y * dt * dt;
        yaw_r_p += noise_y * dt;

        // Update the output predictions Matrix :

        Preds_Sig.col(i) << p_x_p, p_y_p, v_p, yaw_p, yaw_r_p;
   
      
    }


    *Xsig_out = Preds_Sig;






}


void UKF::PredictMeanAndCovariance(const Eigen::MatrixXd& Preds_Sig, Eigen::VectorXd* x_pred, Eigen::MatrixXd* P_pred) {

    /*-----------------------------------------------------------------------------------------------------------------
                        Compute the weights : weight1->The mean Sig , Weight2 -> other sig points
    -------------------------------------------------------------------------------------------------------------------*/

    double weight1 = this->lambda / (this->lambda + this->n_a);
    double weight2 = 1 / (2 * (this->lambda + this->n_a));



    /*-----------------------------------------------------------
                           Compute the mean 
    ------------------------------------------------------------*/
    VectorXd Mean = VectorXd::Zero(this->n_x);
    Mean = weight1 * Preds_Sig.col(0);
    
    for (int i = 1; i <15; i++) {

        Mean += weight2 * Preds_Sig.col(i);

    }

    /*------------------------------------------------------------
                    Compute the Covariance Matrix
    -------------------------------------------------------------*/

    MatrixXd Cov(this->n_x, this->n_x);

    VectorXd residual0 = Preds_Sig.col(0) - Mean;
    //Normalize the yaw angle between -pi pi
    while (residual0(3) > pi) residual0(3) -= 2. * pi;
    while (residual0(3) < -pi) residual0(3) += 2. * pi;

    Cov = weight1 * (residual0) * (residual0).transpose();
    
    for (int i = 1; i < 15; i++) {

        VectorXd residual = Preds_Sig.col(i) - Mean;

        //Normalize the yaw angle between -pi pi
        while (residual(3) > pi) residual(3) -= 2. * pi;
        while (residual(3) < -pi) residual(3) += 2. * pi;
        
        Cov+= weight2 * (residual) * (residual).transpose();
    }

    *x_pred = Mean;
    *P_pred = Cov;


}


void UKF::PredictRadarMeasurement(const Eigen::MatrixXd& sig_pts, Eigen::VectorXd* z_out, Eigen::MatrixXd* S_out,Eigen::MatrixXd* sig_meas) {


    //Prepare the weights:

    double weight1 = this->lambda / (this->lambda + this->n_a);
    double weight2 = 1 / (2 * (this->lambda + this->n_a));

    //Create matrix that holds sigma points in measurment space:

    MatrixXd Sig_meas(this->n_z, 2 * this->n_a + 1);

    //Create vector and covariance matrix of the measurement :

    VectorXd z_meas=VectorXd::Zero(this->n_z);
    MatrixXd S=MatrixXd::Zero(this->n_z, this->n_z);

   

    /*-------------------------------------------------------------------------------------------------------
                                   Compute the mean of the measurement sigma points
    --------------------------------------------------------------------------------------------------------*/
    double weight;
    for (int i = 0; i < 2 * this->n_a + 1; i++) {

        double p_x = sig_pts(0,i);
        double p_y = sig_pts(1, i);
        double v = sig_pts(2, i);
        double yaw = sig_pts(3, i);
        double yaw_r = sig_pts(4, i);

        Sig_meas(0, i) = sqrt(p_x * p_x + p_y * p_y);//radius
        Sig_meas(1, i) = atan2(p_y, p_x);//angle
        Sig_meas(2, i) = (p_x * cos(yaw)*v + p_y * sin(yaw)*v) / sqrt(p_x * p_x + p_y * p_y);   //radius_velocity


   

        // Set the appropriate weights :
        if (i == 0)weight = weight1;
        else weight = weight2;

        //Compute the mean of the measurement sigma points
        z_meas += weight * Sig_meas.col(i);

    }

    cout << "mean done !" << endl;

    /*-------------------------------------------------------------------------------------------------------
                              Compute the covariance matrix of measurement sigma points                         
    -------------------------------------------------------------------------------------------------------- */

    for (int i = 0; i < 2 * this->n_a + 1; i++) {

        VectorXd residual = Sig_meas.col(i) - z_meas;

        // Normalize the angle between [-pi pi]
        while (residual(1) > pi) residual(1) -= 2. * pi;
        while (residual(1) < -pi) residual(1) += 2. * pi;

        //Choose the right weight;
        if (i == 0) weight = weight1;
        else weight = weight2;

        //Compute the covariance matrix of the measurement sigma points
        S += weight * residual * residual.transpose();
    }

    cout << "covariance matrix done !" << endl;
    /*---------------------------------------------------------------------------------------------------------
                         Add measurement noise matrix R to the measurement covariance matrix
    ----------------------------------------------------------------------------------------------------------*/
    
    
    MatrixXd R = MatrixXd(n_z, n_z);
    R << this->std_r * this->std_r  ,            0                       ,        0                ,
                     0              , this->std_phi* this->std_phi       ,        0                ,
                     0              ,            0                       , this->std_v* this->std_v;
    
    S = S + R;


    *S_out = S;
    *z_out = z_meas;
    *sig_meas = Sig_meas;


}

void UKF::CrossCorrelationT(const Eigen::MatrixXd& sig_state, const Eigen::VectorXd& x_state, const Eigen::VectorXd& x_meas, const Eigen::MatrixXd& sig_meas, Eigen::MatrixXd* T) {

    //Prepare the weights:

    double weight1 = this->lambda / (this->lambda + this->n_a);
    double weight2 = 1 / (2 * (this->lambda + this->n_a));

    double weight;
    MatrixXd TT = MatrixXd::Zero(this->n_x, this->n_z);
    for (int i = 0; i < 2 * this->n_a + 1; i++) {

        if (i == 0)weight = weight1;
        else weight = weight2;

        VectorXd state_residual = sig_state.col(i) - x_state;
        // angle normalization
        while (state_residual(3) > pi) state_residual(3) -= 2 * pi;
        while (state_residual(3) < -pi) state_residual(3) += 2 * pi;

        VectorXd meas_residual = sig_meas.col(i) - x_meas;
        // angle normalization
        while (meas_residual(1) > pi) meas_residual(1) -= 2 * pi;
        while (meas_residual(1) < -pi) meas_residual(1) += 2 * pi;
        
        TT += weight * state_residual * meas_residual.transpose();


    }

    *T = TT;
}

void UKF::UpdateState(const Eigen::VectorXd& x_state, const Eigen::MatrixXd& P_state, const Eigen::VectorXd& measurement, const Eigen::VectorXd& x_meas,
    const Eigen::MatrixXd& S_meas, const Eigen::MatrixXd& T, Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out) {


   /*---------------------------------------------------------
                            Kalman Gain :   
   -----------------------------------------------------------*/
    MatrixXd KG(this->n_x, this->n_z);
    KG = T * S_meas.inverse();

    /*---------------------------------------------------------
                            State Update:
    ----------------------------------------------------------*/
    VectorXd z_residual = measurement - x_meas;
    // angle normalization
    
    cout << "Z_residual = \n" << z_residual << endl;

    while (z_residual(1) > pi) z_residual(1)-= 2. * pi;
    while (z_residual(1) < -pi) z_residual(1)+= 2. * pi;

    cout << "Z_residual = \n" << z_residual << endl;

    *x_out = x_state + KG * z_residual;
    *P_out = P_state - KG * S_meas * KG.transpose();

}