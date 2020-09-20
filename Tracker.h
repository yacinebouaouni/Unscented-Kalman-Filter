#pragma once
#ifndef TRACKER_H_
#define TRACKER_H_

#include <vector>
#include <string>
#include <fstream>
#include "kalman.h"
#include "measurement.h"

class Tracker {

	public:

		Tracker();

		virtual ~Tracker();

		void ProcessMeasurement(const Measurement& measurement_pack)noexcept;

		KalmanFilter kf_;

	private:

		bool is_initialized_;
		int64_t previous_timestamp_;

		//acceleration noise components
		float noise_ax;
		float noise_ay;
};

#endif  // TRACKER_H_