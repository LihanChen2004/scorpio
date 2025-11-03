// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work Copyright (c) 2019, NXROBO Ltd.

#ifndef SCORPIO_BASE__KALMAN_FILTER_HPP_
#define SCORPIO_BASE__KALMAN_FILTER_HPP_

namespace scorpio_base
{

class KalmanFilter
{
public:
  KalmanFilter() : x_est_last_(0.0), p_last_(0.0), q_(0.001), r_(0.1) {}

  KalmanFilter(double q, double r) : x_est_last_(0.0), p_last_(0.0), q_(q), r_(r) {}

  double predict(double measurement)
  {
    // Predict
    double x_temp_est = x_est_last_;
    double p_temp = p_last_ + q_;

    // Update (Kalman gain)
    double k = p_temp / (p_temp + r_);

    // Correct
    double x_est = x_temp_est + k * (measurement - x_temp_est);
    double p = (1.0 - k) * p_temp;

    // Update state
    p_last_ = p;
    x_est_last_ = x_est;

    return x_est;
  }

  void reset()
  {
    x_est_last_ = 0.0;
    p_last_ = 0.0;
  }

private:
  double x_est_last_;  // Previous estimate
  double p_last_;      // Previous covariance
  double q_;           // Process noise
  double r_;           // Measurement noise
};

}  // namespace scorpio_base

#endif  // SCORPIO_BASE__KALMAN_FILTER_HPP_
