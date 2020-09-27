#pragma once
#ifndef UKF_H
#define UKF_H

#include<Eigen/Dense>

class UKF {

    private:

        int n_x = 5;
        int n_a = 7;
        double lambda = 3 - (double)n_a;

    public:

        /**
         * Constructor
         */
        UKF();

        /**
         * Destructor
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

        void PredictRadarMeasurement(Eigen::VectorXd* z_out,
            Eigen::MatrixXd* S_out);
        void UpdateState(Eigen::VectorXd* x_out,
            Eigen::MatrixXd* P_out);
};

#endif  // UKF_H