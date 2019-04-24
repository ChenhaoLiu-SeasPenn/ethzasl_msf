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

#ifndef ATTITUDE_MEASUREMENT_HPP_
#define ATTITUDE_MEASUREMENT_HPP_

#include <msf_core/msf_types.h>
#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_updates/PoseDistorter.h>
#include <sensor_fusion_comm/AttitudeWithCovarianceStamped.h>
#include <math.h>

namespace msf_updates {
namespace attitude_measurement {
enum {
  nMeasurements = 3
};
/**
 * \brief A measurement as provided by a pose tracking algorithm.
 */
typedef msf_core::MSF_Measurement<sensor_fusion_comm::AttitudeWithCovarianceStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> AttitudeMeasurementBase;

template<
    //int StateLIdx = EKFState::StateDefinition_T::L,
    //int StateQimIdx = EKFState::StateDefinition_T::q_im
    int StatePipIdx = EKFState::StateDefinition_T::p_ip
    //int StateQwvIdx = EKFState::StateDefinition_T::q_wv,
    //int StatePwvIdx = EKFState::StateDefinition_T::p_wv
    >
struct AttitudeMeasurement : public AttitudeMeasurementBase {
 private:
  typedef AttitudeMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
	Eigen::Matrix<double, nMeasurements, 1> m;
	m << msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z;
        m_ = m / m.norm();

    //if (distorter_) {
    //  static double tlast = 0;
    //  if (tlast != 0) {
    //    double dt = time - tlast;
    //    distorter_->Distort(z_p_, z_q_, dt);
    //  }
    //  tlast = time;
    //}

    if (fixed_covariance_) {  // Take fix covariance from reconfigure GUI.
      const double s_m = n_m_;
      R_ =
          (Eigen::Matrix<double, nMeasurements, 1>() << s_m, s_m, s_m)
              .finished().asDiagonal();
    } else {  // Take covariance from sensor. // Note this is NOT implemented!!!!!
      R_.block<nMeasurements, nMeasurements>(0, 0) = Eigen::Matrix<double, nMeasurements, nMeasurements>(
          &msg->magnetic_field_covariance[0]);

      if (msg->header.seq % 100 == 0) {  // Only do this check from time to time.
        if (R_.block<nMeasurements, nMeasurements>(0, 0).determinant() < -0.001)
          MSF_WARN_STREAM_THROTTLE(
              60,
              "The covariance matrix you provided for " "the pose sensor is not positive definite: "<<(R_.block<nMeasurements, nMeasurements>(0, 0)));
      }

      // Clear cross-correlations between q and p.
      //R_.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Zero();
      //R_.block<3, 3>(3, 0) = Eigen::Matrix<double, 3, 3>::Zero();
      //R_(6, 6) = 1e-6;  // q_wv yaw-measurement noise

      /*************************************************************************************/
      // Use this if your pose sensor is ethzasl_ptam (www.ros.org/wiki/ethzasl_ptam)
      // ethzasl_ptam publishes the camera pose as the world seen from the camera.
	  // NOTE: Magnetometer IS a world sensor
      //if (!measurement_world_sensor_) {
      //  Eigen::Matrix<double, 3, 3> C_zq = z_q_.toRotationMatrix();
      //  z_q_ = z_q_.conjugate();
      //  z_p_ = -C_zq.transpose() * z_p_;

      //  Eigen::Matrix<double, 6, 6> C_cov(Eigen::Matrix<double, 6, 6>::Zero());
      //  C_cov.block<3, 3>(0, 0) = C_zq;
      //  C_cov.block<3, 3>(3, 3) = C_zq;

      //  R_.block<6, 6>(0, 0) = C_cov.transpose() * R_.block<6, 6>(0, 0) * C_cov;
      //}
      /*************************************************************************************/
    }
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, nMeasurements, 1> m_;  /// Magnetometer seen from world.
  double n_m_, incl_, decl_;  /// Elevation and azimuth measurement noise.

  bool measurement_world_sensor_;
  bool fixed_covariance_;
  msf_updates::PoseDistorter::Ptr distorter_;
  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  //enum AuxState {
  //  L = StateLIdx,
  //  q_ic = StateQicIdx,
  //  p_ic = StatePicIdx,
  //  q_wv = StateQwvIdx,
  //  p_wv = StatePwvIdx
  //};

  /*enum AuxState {
	  q_im = StateQimIdx
  };*/

  virtual ~AttitudeMeasurement() {}
  AttitudeMeasurement(double n_m, double incl, double decl,
                  bool fixed_covariance, bool isabsoluteMeasurement,
                  int sensorID, int fixedstates, bool enable_mah_outlier_rejection,
                  double mah_threshold, 
                  msf_updates::PoseDistorter::Ptr distorter =
                      msf_updates::PoseDistorter::Ptr())
      : AttitudeMeasurementBase(isabsoluteMeasurement, sensorID,
                            enable_mah_outlier_rejection, mah_threshold),
        n_m_(n_m),
	incl_(incl),
	decl_(decl),
        measurement_world_sensor_(true),
        fixed_covariance_(fixed_covariance),
        distorter_(distorter),
        fixedstates_(fixedstates) {}
  virtual std::string Type() {
    return "attitude";
  }

  /// Borrow from this: https://github.com/ethz-asl/ethzasl_msf/blob/vismaggps/vismaggps_fusion/src/mag_dir.cpp
  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    //Eigen::Matrix<double, 3, 3> C_wv = state.Get<StateQwvIdx>()
    //    .toRotationMatrix();
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .toRotationMatrix();

    //Eigen::Matrix<double, 3, 3> C_mi = state.Get<StateQimIdx>()
    //    .conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    Eigen::Matrix<double, 3, 1> vecold;
	Eigen::Matrix<double, 3, 1> vecold_dalpha;
	Eigen::Matrix<double, 3, 1> vecold_dbeta;

    //vecold = (-state.Get<StatePwvIdx>() + state.Get<StateDefinition_T::p>()
    //    + C_q * state.Get<StatePicIdx>()) * state.Get<StateLIdx>();

	Eigen::Matrix<double, 1, 1> beta = state.Get<StateDefinition_T::beta>();
	Eigen::Matrix<double, 1, 1> alpha = state.Get<StateDefinition_T::alpha>();
	double beta_d = beta(0, 0);
	double alpha_d = alpha(0, 0);
	vecold << sin(beta_d)*cos(alpha_d), cos(beta_d)*cos(alpha_d), sin(alpha_d);
	vecold_dalpha << -sin(beta_d)*sin(alpha_d), -cos(beta_d)*sin(alpha_d), cos(alpha_d);
	vecold_dbeta << cos(beta_d)*cos(alpha_d), -sin(beta_d)*cos(alpha_d), 0;

    Eigen::Matrix<double, 3, 3> skewold = Skew(C_q * vecold);
    //Eigen::Matrix<double, 3, 3> skewold_2 = Skew(C_mi * C_q * vecold);

    //Eigen::Matrix<double, 3, 3> pci_sk = Skew(state.Get<StatePicIdx>());


    // Get indices of states in error vector.
    enum {
      kIdxstartcorr_p = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      kIdxstartcorr_v = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      kIdxstartcorr_q = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,

      //kIdxstartcorr_L = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
      //    StateLIdx>::value,
      //kIdxstartcorr_qwv = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
      //    StateQwvIdx>::value,
      //kIdxstartcorr_pwv = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
      //    StatePwvIdx>::value,
      //kIdxstartcorr_qim = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
      //    StateQimIdx>::value,
	  kIdxstartcorr_alpha = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
		  StateDefinition_T::alpha>::value,
	  kIdxstartcorr_beta = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
		  StateDefinition_T::beta>::value,
      //kIdxstartcorr_pic = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
      //    StatePicIdx>::value,
    };

    // Read the fixed states flags.
    //bool scalefix = (fixedstates_ & 1 << StateLIdx);
    //bool calibposfix = (fixedstates_ & 1 << StatePicIdx);
    //bool calibattfix = (fixedstates_ & 1 << StateQimIdx);
    //bool driftwvattfix = (fixedstates_ & 1 << StateQwvIdx);
    //bool driftwvposfix = (fixedstates_ & 1 << StatePwvIdx);

    // Set crosscov to zero for fixed states.
    //if (scalefix)
    //  state_in->ClearCrossCov<StateLIdx>();
    //if (calibposfix)
    //  state_in->ClearCrossCov<StatePicIdx>();
    //if (calibattfix)
    //  state_in->ClearCrossCov<StateQimIdx>();
    //if (driftwvattfix)
    //  state_in->ClearCrossCov<StateQwvIdx>();
    //if (driftwvposfix)
    //  state_in->ClearCrossCov<StatePwvIdx>();


    // Construct H matrix.
	H.block<3, 3>(0, kIdxstartcorr_q) = skewold;
	//H.block<3, 3>(0, kIdxstartcorr_qim) = skewold_2;
	H.block<3, 1>(0, kIdxstartcorr_alpha) = C_q * vecold_dalpha;
	H.block<3, 1>(0, kIdxstartcorr_beta) = C_q * vecold_dbeta;

 //   // Position:
	//H.block<3, 3>(0, kIdxstartcorr_p) = C_wv;
 //       * state.Get<StateLIdx>()(0);  // p

	//H.block<3, 3>(0, kIdxstartcorr_q) = C_q;

 //   //H.block<3, 3>(0, kIdxstartcorr_q) = -C_wv * C_q * pci_sk
 //   //    * state.Get<StateLIdx>()(0);  // q

 //   //H.block<3, 1>(0, kIdxstartcorr_L) =
 //   //    scalefix ?
 //   //        Eigen::Matrix<double, 3, 1>::Zero() :
 //   //        (C_wv * C_q * state.Get<StatePicIdx>() + C_wv
 //   //                * (-state.Get<StatePwvIdx>()
 //   //                    + state.Get<StateDefinition_T::p>())).eval();  // L

 //   H.block<3, 3>(0, kIdxstartcorr_qwv) =
 //       driftwvattfix ?
 //           Eigen::Matrix<double, 3, 3>::Zero() : (-C_wv * skewold).eval();  // q_wv

 //   H.block<3, 3>(0, kIdxstartcorr_pic) =
 //       calibposfix ?
 //           Eigen::Matrix<double, 3, 3>::Zero() :
 //           (C_wv * C_q * state.Get<StateLIdx>()(0)).eval();  //p_ic


 //   // TODO (slynen): Check scale commenting
 //   H.block<3, 3>(0, kIdxstartcorr_pwv) =
 //       driftwvposfix ?
 //           Eigen::Matrix<double, 3, 3>::Zero() :
 //           (-Eigen::Matrix<double, 3, 3>::Identity()
 //           /* * state.Get<StateLIdx>()(0)*/).eval();  //p_wv

 //   // Attitude.
 //   H.block<3, 3>(3, kIdxstartcorr_q) = C_ci;  // q

 //   H.block<3, 3>(3, kIdxstartcorr_qwv) =
 //       driftwvattfix ?
 //           Eigen::Matrix<double, 3, 3>::Zero() :
 //           (C_ci * C_q.transpose()).eval();  // q_wv

 //   H.block<3, 3>(3, kIdxstartcorr_qic) =
 //       calibattfix ?
 //           Eigen::Matrix<double, 3, 3>::Zero() :
 //           Eigen::Matrix<double, 3, 3>::Identity().eval();  //q_ic

    // This line breaks the filter if a position sensor in the global frame is
    // available or if we want to set a global yaw rotation.
    //H.block<1, 1>(6, kIdxstartcorr_qwv + 2) = Eigen::Matrix<double, 1, 1>::
    // Constant(driftwvattfix ? 0.0 : 1.0); // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object
   */
  /// Calculate r_ and apply correction
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
      // or is is just relative to the last measurement.
      // Get a const ref, so we can read core states
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      //Eigen::Matrix<double, 3, 3> C_wv = state.Get<StateQwvIdx>()
      //    .conjugate().toRotationMatrix();

      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();
	  //Eigen::Matrix<double, 3, 3> C_mi = state.Get<StateQimIdx>()
		//  .conjugate().toRotationMatrix();
	  Eigen::Matrix<double, 3, 1> vecold;
	Eigen::Matrix<double, 1, 1> beta = state.Get<StateDefinition_T::beta>();
	Eigen::Matrix<double, 1, 1> alpha = state.Get<StateDefinition_T::alpha>();
	double beta_d = beta(0, 0);
	double alpha_d = alpha(0, 0);
	  vecold << sin(beta_d)*cos(alpha_d), cos(beta_d)*cos(alpha_d), sin(alpha_d);
	  r_old.block<3, 1>(0, 0) = m_ - C_q * vecold;


      // Construct residuals.
      // Position.
      //r_old.block<3, 1>(0, 0) = z_p_
      //    - (C_wv.transpose()
      //        * (-state.Get<StatePwvIdx>()
      //            + state.Get<StateDefinition_T::p>()
      //            + C_q.transpose() * state.Get<StatePicIdx>()))
      //        * state.Get<StateLIdx>();

      //// Attitude.
      //Eigen::Quaternion<double> q_err;
      //q_err = (state.Get<StateQwvIdx>()
      //    * state.Get<StateDefinition_T::q>()
      //    * state.Get<StateQicIdx>()).conjugate() * z_q_;
      //r_old.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
      //// Vision world yaw drift.
      //q_err = state.Get<StateQwvIdx>();

      //r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
      //    / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }

      // Call update step in base class.
      MSF_INFO_STREAM("!!!!!!!!Applying update with magnetometer data. Measurement:" << m_ <<"Residual: " << r_old);
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } else {
		MSF_ERROR_STREAM("Failed: Magnetometer is NOT a relavent measure. Method not implemented."
			"Measurement discarded.");
      // Init variables: Get previous measurement.
    //  shared_ptr < msf_core::MSF_MeasurementBase<EKFState_T> > prevmeas_base =
    //      core.GetPreviousMeasurement(this->time, this->sensorID_);

    //  if (prevmeas_base->time == msf_core::constants::INVALID_TIME) {
    //    MSF_WARN_STREAM(
    //        "The previous measurement is invalid. Could not apply measurement! " "time:"<<this->time<<" sensorID: "<<this->sensorID_);
    //    return;
    //  }

    //  // Make this a pose measurement.
    //  shared_ptr<PoseMeasurement> prevmeas = dynamic_pointer_cast
    //      < PoseMeasurement > (prevmeas_base);
    //  if (!prevmeas) {
    //    MSF_WARN_STREAM(
    //        "The dynamic cast of the previous measurement has failed. "
    //        "Could not apply measurement");
    //    return;
    //  }

    //  // Get state at previous measurement.
    //  shared_ptr<EKFState_T> state_nonconst_old = core.GetClosestState(
    //      prevmeas->time);

    //  if (state_nonconst_old->time == msf_core::constants::INVALID_TIME) {
    //    MSF_WARN_STREAM(
    //        "The state at the previous measurement is invalid. Could "
    //        "not apply measurement");
    //    return;
    //  }

    //  // Get a const ref, so we can read core states.
    //  const EKFState_T& state_new = *state_nonconst_new;
    //  const EKFState_T& state_old = *state_nonconst_old;

    //  Eigen::Matrix<double, nMeasurements,
    //      msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new,
    //      H_old;
    //  Eigen::Matrix<double, nMeasurements, 1> r_new, r_old;

    //  CalculateH(state_nonconst_old, H_old);

    //  H_old *= -1;

    //  CalculateH(state_nonconst_new, H_new);

    //  //TODO (slynen): check that both measurements have the same states fixed!
    //  Eigen::Matrix<double, 3, 3> C_wv_old, C_wv_new;
    //  Eigen::Matrix<double, 3, 3> C_q_old, C_q_new;

    //  C_wv_new = state_new.Get<StateQwvIdx>().conjugate().toRotationMatrix();
    //  C_q_new = state_new.Get<StateDefinition_T::q>().conjugate()
    //      .toRotationMatrix();

    //  C_wv_old = state_old.Get<StateQwvIdx>().conjugate().toRotationMatrix();
    //  C_q_old = state_old.Get<StateDefinition_T::q>().conjugate()
    //      .toRotationMatrix();

    //  // Construct residuals.
    //  // Position:
    //  Eigen::Matrix<double, 3, 1> diffprobpos = (C_wv_new.transpose()
    //      * (-state_new.Get<StatePwvIdx>() + state_new.Get<StateDefinition_T::p>()
    //          + C_q_new.transpose() * state_new.Get<StatePicIdx>()))
    //      * state_new.Get<StateLIdx>() - (C_wv_old.transpose()
    //      * (-state_old.Get<StatePwvIdx>() + state_old.Get<StateDefinition_T::p>()
    //          + C_q_old.transpose() * state_old.Get<StatePicIdx>()))
    //          * state_old.Get<StateLIdx>();


    //  Eigen::Matrix<double, 3, 1> diffmeaspos = z_p_ - prevmeas->z_p_;

    //  r_new.block<3, 1>(0, 0) = diffmeaspos - diffprobpos;

    //  // Attitude:
    //  Eigen::Quaternion<double> diffprobatt = (state_new.Get<StateQwvIdx>()
    //      * state_new.Get<StateDefinition_T::q>()
    //      * state_new.Get<StateQicIdx>()).conjugate()
    //      * (state_old.Get<StateQwvIdx>()
    //          * state_old.Get<StateDefinition_T::q>()
    //          * state_old.Get<StateQicIdx>());

    //  Eigen::Quaternion<double> diffmeasatt = z_q_.conjugate() * prevmeas->z_q_;

    //  Eigen::Quaternion<double> q_err;
    //  q_err = diffprobatt.conjugate() * diffmeasatt;

    //  r_new.block<3, 1>(3, 0) = q_err.vec() / q_err.w() * 2;
    //  // Vision world yaw drift.
    //  q_err = state_new.Get<StateQwvIdx>();

    //  r_new(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y())
    //      / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

    //  if (!CheckForNumeric(r_old, "r_old")) {
    //    MSF_ERROR_STREAM("r_old: "<<r_old);
    //    MSF_WARN_STREAM(
    //        "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
    //  }
    //  if (!CheckForNumeric(H_new, "H_old")) {
    //    MSF_ERROR_STREAM("H_old: "<<H_new);
    //    MSF_WARN_STREAM(
    //        "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
    //  }
    //  if (!CheckForNumeric(R_, "R_")) {
    //    MSF_ERROR_STREAM("R_: "<<R_);
    //    MSF_WARN_STREAM(
    //        "state: "<<const_cast<EKFState_T&>(state_new). ToEigenVector().transpose());
    //  }

    //  // Call update step in base class.
    //  this->CalculateAndApplyCorrectionRelative(state_nonconst_old,
    //                                            state_nonconst_new, core, H_old,
    //                                            H_new, r_new, R_);

    }
  }
};

}  // namespace msf_attitude_sensor
}  // namespace msf_updates
#endif  // ATTITUDE_MEASUREMENT_HPP_
