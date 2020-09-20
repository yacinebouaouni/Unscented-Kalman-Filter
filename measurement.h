#pragma once
#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include<Eigen/Dense>

class Measurement {
public:

    enum SensorType {
        LIDAR, RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_=Eigen::VectorXd(2);

    int64_t timestamp_;

};

#endif  // MEASUREMENT_H_