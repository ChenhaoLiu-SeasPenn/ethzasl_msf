/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ATTITUDE_SENSOR_H
#define ATTITUDE_SENSOR_H

#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/MagneticField.h>
// #include <msf_core/gps_conversion.h>
#include <sensor_fusion_comm/AttitudeWithCovarianceStamped.h>
#include <math.h>

namespace msf_attitude_sensor {

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
class AttitudeSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Matrix<double, 3, 1> m_;  ///< Attitude measurement.
  double n_m_;  ///< Attitude measurement noise.
  double delay_;       ///< Delay to be subtracted from the ros-timestamp of
                       //the measurement provided by this sensor.
  double incl_, decl_; ///< Inclination, declination based on current lon/lat

  //ros::Subscriber subPoseWithCovarianceStamped_;
  //ros::Subscriber subTransformStamped_;

  // Currently only support magnetic field input
  ros::Subscriber subMagneticField_;

  // msf_core::GPSConversion gpsConversion_;
  
  // TODO: Magnetic field to pose conversion
  // Directly do this in callback
  //msf_core::MagConversion magconversion_;

  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

   void ProcessAttitudeMeasurement(
       const sensor_fusion_comm::AttitudeWithCovarianceStampedConstPtr& msg);
  //void MeasurementCallback(const geometry_msgs::PointStampedConstPtr & msg);
  //void MeasurementCallback(const geometry_msgs::TransformStampedConstPtr & msg);
  void MeasurementCallback(const sensor_msgs::MagneticFieldConstPtr& msg);

 public:
  typedef MEASUREMENT_TYPE measurement_t;
  AttitudeSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 1, 1> GetElevationMeasurement() {
	  Eigen::Matrix<double, 1, 1> alpha;
	  double h = hypot(m_(0, 0), m_(1, 0));
	  alpha << incl_;
	  //MSF_INFO_STREAM("!!!!!!Getting init elevation:" << m_(2, 0) << " " << h << " " << atan2(m_(2, 0), h) << "Using message:" << m_);
    return alpha;
  }

  Eigen::Matrix<double, 1, 1> GetAzimuthMeasurement() {
	  Eigen::Matrix<double, 1, 1> beta;
	  beta << decl_;
	  //MSF_INFO_STREAM("!!!!!!Getting init azimuth:"<< m_(1, 0) << " " << m_(0, 0) << " "  << atan2(m_(1, 0), m_(0, 0)) << "decl_" << decl_ << "Using message:" << m_);
	  return beta;
  }
  
  double GetInclination() {return incl_;}
  double GetDeclination() {return decl_;}

  // Setters for configure values.
  void SetNoises(double n_m);
  void SetDelay(double delay);
  void SetInclination(double incl);
  void SetDeclination(double decl);
  // void AdjustGPSZReference(double current_z);
};
}  // namespace msf_attitude_sensor

#include "implementation/attitude_sensorhandler.hpp"

#endif /* ATTITUDE_SENSOR_H */
