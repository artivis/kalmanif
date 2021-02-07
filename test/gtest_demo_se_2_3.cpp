/**
 * \file gtest_demo_se_2_3.cpp
 *
 * This file is a duplicate of file examples/demo_se_2_3.cpp.
 * See examples/demo_se_2_3.cpp for actual documentation.
 */

#include <kalmanif/kalmanif.h>
#include <kalmanif/system_models/simple_imu_system_model.h>
#include "../examples/utils/rand.h"

#include <manif/SE_2_3.h>
#include <manif/gtest/gtest_manif_utils.h>

#include <vector>

using namespace kalmanif;
using namespace manif;

using State = SE_2_3d;
using SystemModel = SimpleImuSystemModel<State::Scalar>;
using Control = SystemModel::Control;
using MeasurementModel = Landmark3DMeasurementModel<State>;
using Landmark = MeasurementModel::Landmark;
using Measurement = MeasurementModel::Measurement;
using StateCovariance = Eigen::Matrix<double, 9, 9>;
using Array6d = Eigen::Array<double, 6, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

using EKF = ExtendedKalmanFilter<State>;
using SEKF = SquareRootExtendedKalmanFilter<State>;
using IEKF = InvariantExtendedKalmanFilter<State>;
using UKFM = UnscentedKalmanFilterManifolds<State>;

TEST(TEST_DEMO_SE_2_3, TEST_DEMO_SE_2_3)
{
  // START CONFIGURATION

  constexpr int control_freq = 100;           // Hz
  constexpr double dt = 1./control_freq;      // s
  constexpr int landmark_freq = 100;          // Hz

  // acceleration due to gravity in world frame
  Vector3d g;
  g << 0, 0, -9.80665;

  State X_simulation = State::Identity();

  // IMU measurements in IMU frame
  Vector3d alpha, alpha_const, omega, alpha_prev, omega_prev;
  alpha_const << 0.1, 0.01, 0.1; // constant acceleration in IMU frame without gravity compensation
  omega << 0.01, 0.1, 0;         // constant angular velocity about x- and y-direction in IMU frame

  // Previous IMU measurements in IMU frame initialized to values expected when stationary
  alpha_prev = alpha = alpha_const - (X_simulation.rotation()).transpose() * g;
  omega_prev << 0, 0, 0;

  // Define a control vector and its noise and covariance
  Control      u_simu, u_est, u_unfilt;

  Vector6d     u_nom, u_noisy, u_noise;
  Array6d      u_sigmas;
  Matrix6d     U;

  u_sigmas << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
  U        = (u_sigmas * u_sigmas).matrix().asDiagonal();

  // Define the beacon's measurements
  Eigen::Vector3d y, y_noise;
  Eigen::Array3d  y_sigmas;
  Eigen::Matrix3d R;

  y_sigmas << 0.01, 0.01, 0.01;
  R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  std::vector<MeasurementModel> measurement_models = {
    MeasurementModel(Landmark(2.0,  0.0,  0.0), R),
    MeasurementModel(Landmark(3.0, -1.0, -1.0), R),
    MeasurementModel(Landmark(2.0, -1.0,  1.0), R),
    MeasurementModel(Landmark(2.0,  1.0,  1.0), R),
    MeasurementModel(Landmark(2.0,  1.0, -1.0), R)
  };

  std::vector<Measurement> measurements(measurement_models.size());

  SystemModel system_model;
  system_model.setCovariance(U);

  StateCovariance state_cov_init = StateCovariance::Zero();
  state_cov_init.block<3, 3>(0, 0) = 0.001 * Matrix3d::Identity();
  state_cov_init.block<3, 3>(3, 3) = 0.01  * Matrix3d::Identity();
  state_cov_init.block<3, 3>(6, 6) = 0.001 * Matrix3d::Identity();

  State X_init = X_simulation;

  EKF ekf(X_init, state_cov_init);

  EXPECT_MANIF_NEAR(X_init, ekf.getState());
  EXPECT_EIGEN_NEAR(state_cov_init, ekf.getCovariance());

  SEKF sekf(X_init, state_cov_init);

  EXPECT_MANIF_NEAR(X_init, sekf.getState());
  EXPECT_EIGEN_NEAR(state_cov_init, sekf.getCovariance());

  IEKF iekf(X_init, state_cov_init);

  EXPECT_MANIF_NEAR(X_init, iekf.getState());
  EXPECT_EIGEN_NEAR(state_cov_init, iekf.getCovariance());

  UKFM ukfm(X_init, state_cov_init);

  EXPECT_MANIF_NEAR(X_init, ukfm.getState());
  EXPECT_EIGEN_NEAR(state_cov_init, ukfm.getCovariance());

  // Make 10 steps. Measure up to three landmarks each time.
  for (double t = 0; t < 30; t += dt) {
    //// I. Simulation

    /// input vector
    u_nom << alpha_prev, omega_prev;

    /// simulate noise
    u_noise = randn(u_sigmas);     // control noise
    u_noisy = u_nom + u_noise;     // noisy control

    u_simu   = u_nom;
    u_est    = u_noisy;

    /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X_simulation = system_model(X_simulation, u_simu, dt);

    /// update expected IMU measurements
    alpha = alpha_const - X_simulation.rotation().transpose() * g; // update expected IMU measurement after moving

    /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
    for (int i = 0; i < measurement_models.size(); ++i)
    {
      auto measurement_model = measurement_models[i];

      y = measurement_model(X_simulation);            // landmark measurement, before adding noise

      /// simulate noise
      y_noise = randn(y_sigmas);                      // measurement noise

      y = y + y_noise;                                // landmark measurement, noisy
      measurements[i] = y;                            // store for the estimator just below
    }

    //// II. Estimation

    /// First we move

    ekf.propagate(system_model, u_est, dt);

    sekf.propagate(system_model, u_est, dt);

    iekf.propagate(system_model, u_est, dt);

    ukfm.propagate(system_model, u_est, dt);

    /// Then we correct using the measurements of each lmk

    if (int(t*100) % int(100./landmark_freq) == 0) {
      for (int i = 0; i < measurement_models.size(); ++i) {
        // landmark
        auto measurement_model = measurement_models[i];

        // measurement
        y = measurements[i];

        // filter update
        ekf.update(measurement_model, y);

        sekf.update(measurement_model, y);

        iekf.update(measurement_model, y);

        ukfm.update(measurement_model, y);
      }
    }

    alpha_prev = alpha;
    omega_prev = omega;

    //// III. Results

    EXPECT_MANIF_NEAR(X_simulation, ekf.getState(), 5e-1);
    EXPECT_MANIF_NEAR(X_simulation, sekf.getState(), 5e-1);
    EXPECT_MANIF_NEAR(X_simulation, iekf.getState(), 5e-1);
    EXPECT_MANIF_NEAR(X_simulation, ukfm.getState(), 1);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
