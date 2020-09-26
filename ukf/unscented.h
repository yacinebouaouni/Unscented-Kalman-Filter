#pragma once
#ifndef UKF_H
#define UKF_H

#include<Eigen/Dense>

class UKF {

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

        /**
         * Student assignment functions
         */
        void GenerateSigmaPoints(const Eigen::VectorXd& x, const Eigen:: MatrixXd& P, Eigen::MatrixXd* Xsig_out);
        void AugmentedSigmaPoints(const Eigen::VectorXd& x,const Eigen::MatrixXd& P, const Eigen::VectorXd& noise, const Eigen::MatrixXd& Q, Eigen::MatrixXd* Xsig_out);
        void SigmaPointPrediction(Eigen::MatrixXd* Xsig_out);
        void PredictMeanAndCovariance(Eigen::VectorXd* x_pred,
            Eigen::MatrixXd* P_pred);
        void PredictRadarMeasurement(Eigen::VectorXd* z_out,
            Eigen::MatrixXd* S_out);
        void UpdateState(Eigen::VectorXd* x_out,
            Eigen::MatrixXd* P_out);
};

#endif  // UKF_H