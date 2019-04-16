/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef POSITION_ATTITUDE_SENSOR_MANAGER_H
#define POSITION_ATTITUDE_SENSOR_MANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/attitude_sensor_handler/attitude_sensorhandler.h>
#include <msf_updates/attitude_sensor_handler/attitude_measurement.h>
#include <msf_updates/position_sensor_handler/position_sensorhandler.h>
#include <msf_updates/position_sensor_handler/position_measurement.h>
#include <msf_updates/PositionAttitudeSensorConfig.h>

namespace msf_updates {

typedef msf_updates::PositionAttitudeSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class PositionAttitudeSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  typedef PositionAttitudeSensorManager this_T;

  // This part should be configured in /include/sensor part
  typedef msf_attitude_sensor::AttitudeSensorHandler<
      msf_updates::attitude_measurement::AttitudeMeasurement<>, this_T> AttitudeSensorHandler_T;
  friend class msf_attitude_sensor::AttitudeSensorHandler<
      msf_updates::attitude_measurement::AttitudeMeasurement<>, this_T>;

  typedef msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T> PositionSensorHandler_T;
  friend class msf_position_sensor::PositionSensorHandler<
      msf_updates::position_measurement::PositionMeasurement, this_T>;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  PositionAttitudeSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/position_attitude_sensor")) {
    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));

    bool distortmeas = false;  ///< Distort the pose measurements

	
    //attitude_handler_.reset(
    //    new AttitudeSensorHandler_T(*this, "", "attitude_sensor", distortmeas));
    //AddHandler(attitude_handler_);

	// Need to resolve this reference
	// Distortion ?
	attitude_handler_.reset(
		new AttitudeSensorHandler_T(*this, "", "attitude_sensor"));
	AddHandler(attitude_handler_);

    position_handler_.reset(
        new PositionSensorHandler_T(*this, "", "position_sensor"));
    AddHandler(position_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(&this_T::Config, this, _1,
                                                    _2);
    reconf_server_->setCallback(f);
  }
  virtual ~PositionAttitudeSensorManager() {
  }

  virtual const Config_T& Getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<AttitudeSensorHandler_T> attitude_handler_;
  shared_ptr<PositionSensorHandler_T> position_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;  ///< Dynamic reconfigure server.

  /**
   * \brief Dynamic reconfigure callback.
   */
 // virtual void Config(Config_T &config, uint32_t level) {
 //   config_ = config;
 //   attitude_handler_->SetNoises(config.attitude_noise_meas_a,
 //                            config.attitude_noise_meas_b);
	//attitude_handler_->SetDelay(config.attitude_delay);

 //   position_handler_->SetNoises(config.position_noise_meas);
 //   position_handler_->SetDelay(config.position_delay);

 //   if ((level & msf_updates::PositionPoseSensor_INIT_FILTER)
 //       && config.core_init_filter == true) {
 //     Init(config.pose_initial_scale);
 //     config.core_init_filter = false;
 //   }

 //   // Init call with "set height" checkbox.
 //   if ((level & msf_updates::PositionPoseSensor_SET_HEIGHT)
 //       && config.core_set_height == true) {
 //     Eigen::Matrix<double, 3, 1> p = pose_handler_->GetPositionMeasurement();
 //     if (p.norm() == 0) {
 //       MSF_WARN_STREAM(
 //           "No measurements received yet to initialize position. Height init "
 //           "not allowed.");
 //       return;
 //     }
 //     double scale = p[2] / config.core_height;
 //     Init(scale);
 //     config.core_set_height = false;
 //   }
 // }

  virtual void Config(Config_T &config, uint32_t level) {
	  config_ = config;
	  attitude_handler_->SetNoises(config.attitude_noise_meas);
	  attitude_handler_->SetDelay(config.attitude_delay);
	  attitude_handler_->SetInclination(config.attitude_inclination);
	  attitude_handler_->SetDeclination(config.attitude_declination);

	  position_handler_->SetNoises(config.position_noise_meas);
	  position_handler_->SetDelay(config.position_delay);

	  if ((level & msf_updates::PositionAttitudeSensor_INIT_FILTER)
		  && config.core_init_filter == true) {
		  Init();
		  config.core_init_filter = false;
	  }

  }

  /// void Init(double scale)
  void Init() const {
    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m, p_ig, p_pos;
	Eigen::Matrix<double, 1, 1> alpha, beta;
    Eigen::Quaternion<double> q, q_im;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;

    // init values
    g << 0, 0, 9.81;	/// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    //q_wv.setIdentity();  // World-vision rotation drift.
    //p_wv.setZero();      // World-vision position drift.

    P.setZero();  // Error state covariance; if zero, a default initialization in msf_core is used.

    p_pos = position_handler_->GetPositionMeasurement();

    //p_vc = pose_handler_->GetPositionMeasurement();
    //q_vc = pose_handler_->GetAttitudeMeasurement();
	alpha = attitude_handler_->GetElevationMeasurement();
	beta = attitude_handler_->GetAzimuthMeasurement();

    MSF_INFO_STREAM(
        "initial measurement magnetometer: elevation:["<<alpha<<"] azimuth: " <<beta);
    MSF_INFO_STREAM(
        "initial measurement position: pos:["<<p_pos.transpose()<<"]");

    // Check if we have already input from the measurement sensor.
    if (!attitude_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize magnetometer elevation and azimuth - "
          "using 0 and 0 respectively");
    if (!position_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize absolute position - using [0 0 0]");

    ros::NodeHandle pnh("~");
    //pnh.param("pose_sensor/init/p_ic/x", p_ic[0], 0.0);
    //pnh.param("pose_sensor/init/p_ic/y", p_ic[1], 0.0);
    //pnh.param("pose_sensor/init/p_ic/z", p_ic[2], 0.0);

    //pnh.param("pose_sensor/init/q_ic/w", q_ic.w(), 1.0);
    //pnh.param("pose_sensor/init/q_ic/x", q_ic.x(), 0.0);
    //pnh.param("pose_sensor/init/q_ic/y", q_ic.y(), 0.0);
    //pnh.param("pose_sensor/init/q_ic/z", q_ic.z(), 0.0);
    //q_ic.normalize();

	pnh.param("attitude_sensor/init/q_im/w", q_im.w(), 1.0);
	pnh.param("attitude_sensor/init/q_im/x", q_im.x(), 0.0);
	pnh.param("attitude_sensor/init/q_im/y", q_im.y(), 0.0);
	pnh.param("attitude_sensor/init/q_im/z", q_im.z(), 0.0);

    //MSF_INFO_STREAM("p_ic: " << p_ic.transpose());
    MSF_INFO_STREAM("q_im: " << STREAMQUAT(q_im));

    pnh.param("position_sensor/init/p_ig/x", p_ig[0], 0.0);
    pnh.param("position_sensor/init/p_ig/y", p_ig[1], 0.0);
    pnh.param("position_sensor/init/p_ig/z", p_ig[2], 0.0);
	MSF_INFO_STREAM("p_ig: " << p_ig.transpose());

    // Calculate initial attitude and position based on sensor measurements
    // here we take the attitude from the pose sensor and augment it with
    // global yaw init.
    //double yawinit = config_.position_yaw_init / 180 * M_PI;

	// Since we have magnetometer as world attitude sensor, the initial yaw could be determined w.r.t. true north
	double yawinit = beta;
    Eigen::Quaterniond yawq(cos(yawinit / 2), 0, 0, sin(yawinit / 2));
    yawq.normalize();

    q = yawq;
    //q_wv = (q * q_ic * q_vc.conjugate()).conjugate();

    MSF_WARN_STREAM("q " << STREAMQUAT(q));
    //MSF_WARN_STREAM("q_wv " << STREAMQUAT(q_wv));

    //Eigen::Matrix<double, 3, 1> p_vision = q_wv.conjugate().toRotationMatrix()
    //    * p_vc / scale - q.toRotationMatrix() * p_ic;

    //TODO (slynen): what if there is no initial position measurement? Then we
    // have to shift vision-world later on, before applying the first position
    // measurement.
    p = p_pos - q.toRotationMatrix() * p_ig;
    //p_wv = p - p_vision;  // Shift the vision frame so that it fits the position
    // measurement

    a_m = q.inverse() * g;			    /// Initial acceleration.

    //TODO (slynen) Fix this.
    //we want z from vision (we did scale init), so:
//    p(2) = p_vision(2);
//    p_wv(2) = 0;

//	  Might need to add this:
//    position_handler_->adjustGPSZReference(p(2));

    // Prepare init "measurement"
    // True means that we will also set the initial sensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    //meas->SetStateInitValue < StateDefinition_T::L
    //    > (Eigen::Matrix<double, 1, 1>::Constant(scale));
    //meas->SetStateInitValue < StateDefinition_T::q_wv > (q_wv);
    //meas->SetStateInitValue < StateDefinition_T::p_wv > (p_wv);
    meas->SetStateInitValue < StateDefinition_T::q_im > (q_im);
    //meas->SetStateInitValue < StateDefinition_T::p_ic > (p_ic);
    meas->SetStateInitValue < StateDefinition_T::p_ig > (p_ig);
	meas->SetStateInitValue < StateDefinition_T::alpha > (alpha);
	meas->SetStateInitValue < StateDefinition_T::beta > (beta);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const {
    // Set scale to 1. 
	// No scale in this configuration.
    //Eigen::Matrix<double, 1, 1> scale;
    //scale << 1.0;
    //state.Set < StateDefinition_T::L > (scale);
	  UNUSED(state);
  }
  virtual void InitState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    //const msf_core::Vector3 nqwvv = msf_core::Vector3::Constant(
    //    config_.pose_noise_q_wv);
    //const msf_core::Vector3 npwvv = msf_core::Vector3::Constant(
    //    config_.pose_noise_p_wv);
    const msf_core::Vector3 nqimv = msf_core::Vector3::Constant(
        config_.pose_noise_q_im);
    const msf_core::Vector3 npigv = msf_core::Vector3::Constant(
        config_.pose_noise_p_ig);
    //const msf_core::Vector1 n_L = msf_core::Vector1::Constant(
    //    config_.pose_noise_scale);

    // Compute the blockwise Q values and store them with the states,
    // these then get copied by the core to the correct places in Qd.
    //state.GetQBlock<StateDefinition_T::L>() = (dt * n_L.cwiseProduct(n_L))
    //    .asDiagonal();
    //state.GetQBlock<StateDefinition_T::q_wv>() =
    //    (dt * nqwvv.cwiseProduct(nqwvv)).asDiagonal();
    //state.GetQBlock<StateDefinition_T::p_wv>() =
    //    (dt * npwvv.cwiseProduct(npwvv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::q_im>() =
        (dt * nqicv.cwiseProduct(nqicv)).asDiagonal();
    state.GetQBlock<StateDefinition_T::p_ig>() =
        (dt * npicv.cwiseProduct(npicv)).asDiagonal();
  }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {

    UNUSED(buffstate);
    UNUSED(correction);

    //const EKFState_T& state = delaystate;
    //if (state.Get<StateDefinition_T::L>()(0) < 0) {
    //  MSF_WARN_STREAM_THROTTLE(
    //      1,
    //      "Negative scale detected: " << state.Get<StateDefinition_T::L>()(0) << ". Correcting to 0.1");
    //  Eigen::Matrix<double, 1, 1> L_;
    //  L_ << 0.1;
    //  delaystate.Set < StateDefinition_T::L > (L_);
    }
  }
};
}
#endif  // POSITION_ATTITUDE_SENSOR_MANAGER_H
