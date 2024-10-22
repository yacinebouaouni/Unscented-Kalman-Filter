#pragma once
#ifndef UKF_H
#define UKF_H

#include<Eigen/Dense>
constexpr auto pi = 3.141592653589793238462643383279502884197;
class UKF {

    private:

        int n_x = 5;
        int n_a = 7;
        int n_z = 3;//Radar measurement size 

        double lambda = 3 - (double)n_a;


        /*------------------------------------------------
            Radar measurement noise standard deviation
         ---------------------------------------------- */
        double std_r = 0.3;//radius in m
        double std_phi = 0.0175;//Angle in rad
        double std_v = 0.1;// radius velocity m/s

    public:

        /*
          Constructor
         */
        UKF();

        /*
          Destructor
         */
        virtual ~UKF();

        /**
         * Init Initializes Unscented Kalman filter
         */
        void Init();

       
        void GenerateSigmaPoints(const Eigen::VectorXd& x, const Eigen:: MatrixXd& P, Eigen::MatrixXd* Xsig_out);
        void AugmentedSigmaPoints(const Eigen::VectorXd& x,const Eigen::MatrixXd& P, const Eigen::VectorXd& noise, const Eigen::MatrixXd& Q, Eigen::MatrixXd* Xsig_out);
        void SigmaPointPrediction(const Eigen::MatrixXd& SigPoints,Eigen::MatrixXd* Xsig_out);
        
        void PredictMeanAndCovariance(const Eigen::MatrixXd& Preds_Sig,Eigen::VectorXd* x_pred,Eigen::MatrixXd* P_pred);

        void PredictRadarMeasurement(const Eigen::MatrixXd& Sig_pts,Eigen::VectorXd* z_out,Eigen::MatrixXd* S_out, Eigen::MatrixXd* sig_meas);

        void CrossCorrelationT(const Eigen::MatrixXd& sig_state, const Eigen::VectorXd& x_state, const Eigen::VectorXd& x_meas, const Eigen::MatrixXd& sig_meas,Eigen::MatrixXd* T);
        void UpdateState(const Eigen::VectorXd& x_state, const Eigen::MatrixXd& P_state, const Eigen::VectorXd& measurement, const Eigen::VectorXd& x_meas,
            const Eigen::MatrixXd& S_meas, const Eigen::MatrixXd& T, Eigen::VectorXd* x_out, Eigen::MatrixXd* P_out);
};

#endif  // UKF_H