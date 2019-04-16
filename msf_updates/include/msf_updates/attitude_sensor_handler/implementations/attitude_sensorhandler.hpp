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

#ifndef ATTITUDE_SENSORHANDLER_HPP_
#define ATTITUDE_SENSORHANDLER_HPP_
#include <math.h>
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>
// #include <msf_core/gps_conversion.h>
// #include <msf_core/mag_conversion.h>

namespace msf_attitude_sensor {
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
AttitudeSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AttitudeHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_m_(1e-6),
      delay_(0) {
  ros::NodeHandle pnh("~/attitude_sensor");
  pnh.param("attitude_use_fixed_covariance", use_fixed_covariance_, false);
  pnh.param("attitude_absolute_measurements", provides_absolute_measurements_,
            false);
  //pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  //pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  MSF_INFO_STREAM_COND(use_fixed_covariance_, "Magnetic field sensor is using fixed "
                       "covariance");
  MSF_INFO_STREAM_COND(!use_fixed_covariance_, "Magnetic field sensor is using "
                       "covariance from sensor");

  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Magnetic field sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Magnetic field sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  //subPointStamped_ =
  //    nh.subscribe<geometry_msgs::PointStamped>
  //("position_input", 20, &MagneticFieldSensorHandler::MeasurementCallback, this);
  //subTransformStamped_ =
  //    nh.subscribe<geometry_msgs::TransformStamped>
  //("transform_input", 20, &MagneticFieldSensorHandler::MeasurementCallback, this);
  subMagneticField_ =
      nh.subscribe<sensor_msgs::MagneticField>
  ("mageticfield_input", 20, &AttitudeSensorHandler::MeasurementCallback, this);

  //z_q_.setZero();

  m_ << 0, 0, 0;

}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AttitudeSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetNoises(
    double n_m) {
	n_m_ = n_m;
}

template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AttitudeSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::SetDelay(
    double delay) {
  delay_ = delay;
}

// template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
// void MagneticFieldSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::AdjustGPSZReference(
//     double current_z) {
//   gpsConversion_.AdjustReference(z_p_(2) - current_z);
// }

// TODO: Fix this after finishing measurement code
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AttitudeSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::ProcessMagneticFieldMeasurement(
    const sensor_fusion_comm::PoseWithCovarianceStampedConstPtr& msg) {
  received_first_measurement_ = true;

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");
  // Do not exceed the 32 bits of int.

  if (!use_fixed_covariance_ && msg->alpha_covariance(0, 0) == 0)  // Take covariance from sensor.
      {
    MSF_WARN_STREAM_THROTTLE(
        2, "Provided message type without covariance but set "
        "fixed_covariance=false at the same time. Discarding message.");
    return;
  }

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  // if (mngr) {
  //   if (mngr->Getcfg().position_fixed_p_ip) {
  //     fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
  //   }
  // }


  // TODO: Dynamic configure for magnetometers
  if (mngr) {
    if (mngr->Getcfg().pose_fixed_q_im) {
      fixedstates |= 1 << MEASUREMENT_TYPE::AuxState::q_im;
    }
  }

  shared_ptr<MEASUREMENT_TYPE> meas(new MEASUREMENT_TYPE(
      n_m_, use_fixed_covariance_, provides_absolute_measurements_,
      this->sensorID, fixedstates, enable_mah_outlier_rejection_,
      mah_threshold_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  m_ = meas->m_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

//template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
//void AttitudeSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
//    const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg) {
//  this->SequenceWatchDog(msg->header.seq, subPoseWithCovarianceStamped_.getTopic());
//
//  MSF_INFO_STREAM_ONCE(
//      "*** position sensor got first measurement from topic "
//          << this->topic_namespace_ << "/" << subPoseStamped_.getTopic()
//          << " ***");
//
//  // sensor_fusion_comm::PointWithCovarianceStampedPtr pointwCov(
//  //     new sensor_fusion_comm::PointWithCovarianceStamped);
//  // pointwCov->header = msg->header;
//  // pointwCov->point = msg->point;
//
//  // ProcessPositionMeasurement(pointwCov);
//
//  ProcessPoseMeasurement(msg);
//}
//
//template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
//void MagneticFieldSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
//    const geometry_msgs::TransformStampedConstPtr & msg) {
//  this->SequenceWatchDog(msg->header.seq, subTransformStamped_.getTopic());
//  MSF_INFO_STREAM_ONCE(
//      "*** pose sensor got first measurement from topic "
//          << this->topic_namespace_ << "/" << subTransformStamped_.getTopic()
//          << " ***");
//
//  double time_now = msg->header.stamp.toSec();
//  const double epsilon = 0.001; // Small time correction to avoid rounding errors in the timestamps.
//  if (time_now - timestamp_previous_pose_ <= pose_measurement_minimum_dt_ - epsilon) {
//    MSF_WARN_STREAM_THROTTLE(30, "Pose measurement throttling is on, dropping messages"
//                             "to be below " +
//                             std::to_string(1/pose_measurement_minimum_dt_) + " Hz");
//    return;
//  }
//
//  timestamp_previous_pose_ = time_now;
//
//  geometry_msgs::PoseWithCovarianceStampedPtr pose(
//      new geometry_msgs::PoseWithCovarianceStamped());
//
//  if (!use_fixed_covariance_)  // Take covariance from sensor.
//  {
//    MSF_WARN_STREAM_THROTTLE(
//        2,
//        "Provided message type without covariance but set fixed_covariance == "
//        "false at the same time. Discarding message.");
//    return;
//  }
//
//  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
//  pose->header = msg->header;
//
//  // TODO: find out how to process artificial measurement with unobservable DoFs
//  pose->pose.pose.position.x = msg->transform.translation.x;
//  pose->pose.pose.position.y = msg->transform.translation.y;
//  pose->pose.pose.position.z = msg->transform.translation.z;
//
//  pose->pose.pose.orientation.w = msg->transform.rotation.w;
//  pose->pose.pose.orientation.x = msg->transform.rotation.x;
//  pose->pose.pose.orientation.y = msg->transform.rotation.y;
//  pose->pose.pose.orientation.z = msg->transform.rotation.z;
//
//  ProcessPoseMeasurement(pose);
//}



/// TODO: Inclination/Declination correction
template<typename MEASUREMENT_TYPE, typename MANAGER_TYPE>
void AttitudeSensorHandler<MEASUREMENT_TYPE, MANAGER_TYPE>::MeasurementCallback(
    const sensor_msgs::MagneticFieldConstPtr& msg) {
  this->SequenceWatchDog(msg->header.seq, subMagneticField_.getTopic());

  MSF_INFO_STREAM_ONCE(
      "*** position sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subMagneticField_.getTopic()
          << " ***");

  // Fixed covariance will be set in measurement class -> MakeFromSensorReadingImpl.
  static bool referenceinit = false;  //TODO (slynen): Dynreconf reset reference.
  if (!referenceinit) {
    // TODO: Do the inclination with configuration here
    //magConversion_.InitReference(msg->magnetic_field)
    MSF_WARN_STREAM(
        "Initialized Magnetic Field reference of topic: " << this->topic_namespace_ << "/"
            << subMagneticField_.getTopic() << " to decl/incl: [" << 0
            << ", " << 0 << "]");
    referenceinit = true;
  }

  //Eigen::Matrix<double, 1, 1> alpha, beta;
  //Eigen::Matrix<double, 2, 2> covariance;
  //double h = hypot(msg->magnetic_field(0), msg->magnetic_field(1));
  //alpha << atan2(msg->magnetic_field(2), h);
  //beta << atan2(msg->magnetic_field(1), msg->magnetic_field(0));

  // TODO: Calculate alpha/beta covariance
  //covariance = Eigen::Matrix<double, 2, 2>::Zero();

  sensor_fusion_comm::AttitudeWithCovarianceStampedPtr attwCov(
      new sensor_fusion_comm::AttitudeWithCovarianceStamped);
  attwCov->header = msg->header;

  // Store the ENU data in the position fields.
  attwCov->magnetic_field = msg->magnetic_field;
  attwCov->magnetic_field_covariance = msg->magnetic_field_covariance;
  //attwCov->alpha(0, 0) = alpha(0, 0);
  //attwCov->beta(0, 0) = beta(0, 0);

  //Currently no covariance
  //attwCov->covariance<2, 2>(0, 0) = covariance<2, 2>(0, 0);

  ProcessAttitudeMeasurement(attwCov);
}
}  // namespace msf_attitude_sensor
#endif  // ATTITUDE_SENSORHANDLER_HPP_