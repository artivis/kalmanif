/**
 * \file gtest_demo_se2.cpp
 *
 * This file is a duplicate of file examples/demo_se2.cpp.
 * See examples/demo_se2.cpp for actual documentation.
 */

#include <kalmanif/kalmanif.h>
#include "../examples/utils/rand.h"

#include <manif/SE2.h>
#include <manif/gtest/gtest_manif_utils.h>

#include <vector>

using namespace kalmanif;
using namespace manif;

using State = SE2d;
using StateCovariance = Covariance<State>;
using SystemModel = LieSystemModel<State>;
using Control = SystemModel::Control;
using MeasurementModel = Landmark2DMeasurementModel<State>;
using Landmark = MeasurementModel::Landmark;
using Measurement = MeasurementModel::Measurement;

using EKF = ExtendedKalmanFilter<State>;
using SEKF = SquareRootExtendedKalmanFilter<State>;
using IEKF = InvariantExtendedKalmanFilter<State>;
using UKFM = UnscentedKalmanFilterManifolds<State>;

TEST(TEST_DEMO_SE2, TEST_DEMO_SE2)
{
  // START CONFIGURATION

  constexpr double dt = 0.01;                 // s
  double sqrtdt = std::sqrt(dt);

  constexpr double var_gyro = 1e-3;           // (rad/s)^2
  constexpr double var_wheel_odometry = 9e-5; // (m/s)^2
  constexpr double var_gps = 6e-3;

  constexpr int gps_freq = 100;                // Hz
  constexpr int landmark_freq = 100;           // Hz

  State X_simulation = State::Identity();

  // Define a control vector and its noise and covariance
  Control             u_simu, u_est;

  Eigen::Vector3d     u_nom, u_noisy, u_noise;
  Eigen::Array3d      u_sigmas;
  Eigen::Matrix3d     U;

  u_nom << 0, 0, 0;
  u_sigmas << std::sqrt(var_wheel_odometry), std::sqrt(var_wheel_odometry), std::sqrt(var_gyro);
  U        = (u_sigmas * u_sigmas * 1./dt).matrix().asDiagonal();

  // Define the beacon's measurements
  Eigen::Vector2d y, y_noise;
  Eigen::Array2d  y_sigmas;
  Eigen::Matrix2d R;

  y_sigmas << 0.01, 0.01;
  R        = (y_sigmas * y_sigmas).matrix().asDiagonal();
  y_sigmas << 0, 0;

  std::vector<MeasurementModel> measurement_models = {
    MeasurementModel(Landmark(2.0,  0.0), R),
    MeasurementModel(Landmark(2.0,  1.0), R),
    MeasurementModel(Landmark(2.0, -1.0), R)
  };

  std::vector<Measurement> measurements(measurement_models.size());

  // Define the gps measurements
  Eigen::Vector2d y_gps, y_gps_noise;
  Eigen::Array2d  y_gps_sigmas;
  Eigen::Matrix2d R_gps;

  y_gps_sigmas << std::sqrt(var_gps), std::sqrt(var_gps);
  R_gps        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  SystemModel system_model(U);

  EXPECT_EIGEN_NEAR(U, system_model.getCovariance());

  StateCovariance state_cov_init = StateCovariance::Zero();
  state_cov_init(0, 0) = 0.01;
  state_cov_init(1, 1) = 0.01;
  state_cov_init(2, 2) = MANIF_PI/10.;

  // Eigen::Vector3d n = randn<Eigen::Array3d>();
  // Eigen::Vector3d X_init_coeffs = state_cov_init.cwiseSqrt() * n;
  // State X_init(X_init_coeffs(0), X_init_coeffs(1), X_init_coeffs(2));
  State X_init = State::Identity();

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

    /// simulate noise
    u_noise = randn<Eigen::Array3d>(u_sigmas / sqrtdt); // control noise
    u_noisy = u_nom + u_noise;                          // noisy control

    u_simu   = u_nom   * dt;
    u_est    = u_noisy * dt;

    /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X_simulation = system_model(X_simulation, u_simu);

    /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
    for (int i = 0; i < measurement_models.size(); ++i)
    {
      auto measurement_model = measurement_models[i];

      y = measurement_model(X_simulation);            // landmark measurement, before adding noise

      /// simulate noise
      y_noise = randn(y_sigmas);

      y = y + y_noise;                                // landmark measurement, noisy
      measurements[i] = y;                            // store for the estimator just below
    }

    //// II. Estimation

    /// First we move

    ekf.propagate(system_model, u_est);

    sekf.propagate(system_model, u_est);

    iekf.propagate(system_model, u_est, dt);

    ukfm.propagate(system_model, u_est);

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

    // GPS measurement update
    if (int(t*100) % int(100./gps_freq) == 0) {

      // gps measurement model
      auto gps_measurement_model = DummyGPSMeasurementModel<State>(R_gps);

      y_gps = gps_measurement_model(X_simulation);                  // gps measurement, before adding noise

      /// simulate noise
      y_gps_noise = randn(y_gps_sigmas);                            // measurement noise
      y_gps = y_gps + y_gps_noise;                                  // gps measurement, noisy

      // filter update
      ekf.update(gps_measurement_model, y_gps);

      sekf.update(gps_measurement_model, y_gps);

      iekf.update(gps_measurement_model, y_gps);

      ukfm.update(gps_measurement_model, y_gps);
    }

    //// III. Next iteration

    u_nom << 0.1 * std::cos(t) + 10.0,
             0.0,
             std::exp(-0.03 * (t)) * std::cos(t);

    //// IV. Results

    EXPECT_MANIF_NEAR(X_simulation, ekf.getState(), 5e-1);
    EXPECT_MANIF_NEAR(X_simulation, sekf.getState(), 5e-1);
    EXPECT_MANIF_NEAR(X_simulation, iekf.getState(), 5e-1);
    EXPECT_MANIF_NEAR(X_simulation, ukfm.getState(), 5e-1);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
