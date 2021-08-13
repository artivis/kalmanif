/**
 * \file demo_se3.cpp
 *
 *  Created on: Feb 5, 2021
 *     \author: artivis
 *
 *  ---------------------------------------------------------
 *  This file is:
 *  (c) 2021 artivis
 *
 *  This file is part of `kalmanif`, a C++ template-only library
 *  for Kalman filtering on Lie groups targeted at estimation for robotics.
 *  Manif is:
 *  (c) 2015 mherb
 *  (c) 2021 artivis
 *  ---------------------------------------------------------
 *
 *  ---------------------------------------------------------
 *  Demonstration example:
 *
 *  3D Robot localization based on fixed beacons.
 *
 *  See demo_se2.cpp for the 2D equivalent.
 *  ---------------------------------------------------------
 *
 *  This demo corresponds to the 3D version of the application
 *  in chapter V, section A, in the paper Sola-18,
 *  [https://arxiv.org/abs/1812.01537].
 *
 *  The following is an abstract of the content of the paper.
 *  Please consult the paper for better reference.
 *
 *
 *  We consider a robot in 3D space surrounded by a small
 *  number of punctual landmarks or _beacons_.
 *  The robot receives control actions in the form of axial
 *  and angular velocities, and is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *
 *  The robot pose X is in SE(3) and the beacon positions b_k in R^3,
 *
 *      X = |  R   t |              // position and orientation
 *          |  0   1 |
 *
 *      b_k = (bx_k, by_k, bz_k)    // lmk coordinates in world frame
 *
 *  The control signal u is a twist in se(3) comprising longitudinal
 *  velocity vx and angular velocity wz, with no other velocity
 *  components, integrated over the sampling time dt.
 *
 *      u = (vx*dt, 0, 0, 0, 0, w*dt)
 *
 *  The control is corrupted by additive Gaussian noise u_noise,
 *  with covariance
 *
 *    Q = diagonal(sigma_x^2, sigma_y^2, sigma_z^2, sigma_roll^2, sigma_pitch^2, sigma_yaw^2).
 *
 *  This noise accounts for possible lateral and rotational slippage
 *  through a non-zero values of sigma_y, sigma_z, sigma_roll and sigma_pitch.
 *
 *  At the arrival of a control u, the robot pose is updated
 *  with X <-- X * Exp(u) = X + u.
 *
 *  Landmark measurements are of the range and bearing type,
 *  though they are put in Cartesian form for simplicity.
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice the rigid motion action y = h(X,b) = X^-1 * b
 *  (see appendix D),
 *
 *      y_k = (brx_k, bry_k, brz_k)    // lmk coordinates in robot frame
 *
 *  We consider the beacons b_k situated at known positions.
 *  We define the pose to estimate as X in SE(3).
 *  The estimation error dx and its covariance P are expressed
 *  in the tangent space at X.
 *
 *  All these variables are summarized again as follows
 *
 *    X   : robot pose, SE(3)
 *    u   : robot control, (v*dt; 0; 0; 0; 0; w*dt) in se(3)
 *    Q   : control perturbation covariance
 *    b_k : k-th landmark position, R^3
 *    y   : Cartesian landmark measurement in robot frame, R^3
 *    R   : covariance of the measurement noise
 *
 *  The motion and measurement models are
 *
 *    X_(t+1) = f(X_t, u) = X_t * Exp ( w )     // motion equation
 *    y_k     = h(X, b_k) = X^-1 * b_k          // measurement equation
 *
 *  The algorithm below comprises first a simulator to
 *  produce measurements, then uses these measurements
 *  to estimate the state, using several Kalman filter available
 *  in the library.
 *
 *  Printing simulated state and estimated state together
 *  with an unfiltered state (i.e. without Kalman corrections)
 *  allows for evaluating the quality of the estimates.
 */

#include <kalmanif/kalmanif.h>

#include "utils/rand.h"
#include "utils/plots.h"
#include "utils/utils.h"

#include <manif/SE3.h>

#include <vector>

using namespace kalmanif;
using namespace manif;

using State = SE3d;
using StateCovariance = Covariance<State>;
using SystemModel = LieSystemModel<State>;
using Control = SystemModel::Control;
using MeasurementModel = Landmark3DMeasurementModel<State>;
using Landmark = MeasurementModel::Landmark;
using Measurement = MeasurementModel::Measurement;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Array6d = Eigen::Array<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Filters
using EKF = ExtendedKalmanFilter<State>;
using SEKF = SquareRootExtendedKalmanFilter<State>;
using IEKF = InvariantExtendedKalmanFilter<State>;
using UKFM = UnscentedKalmanFilterManifolds<State>;

// Smoothers
using ERTS = RauchTungStriebelSmoother<EKF>;
using SERTS = RauchTungStriebelSmoother<SEKF>;
using IERTS = RauchTungStriebelSmoother<IEKF>;
using URTSM = RauchTungStriebelSmoother<UKFM>;

int main (int argc, char* argv[]) {

  KALMANIF_DEMO_PROCESS_INPUT(argc, argv);
  KALMANIF_DEMO_PRETTY_PRINT();

  // START CONFIGURATION

  constexpr double eot = 350;                 // s
  constexpr double dt = 0.01;                 // s
  double sqrtdt = std::sqrt(dt);

  constexpr double var_gyro = 1e-4;           // (rad/s)^2
  constexpr double var_odometry = 9e-6;       // (m/s)^2

  constexpr int gps_freq = 10;                // Hz
  constexpr int landmark_freq = 50;           // Hz
  (void)landmark_freq;
  (void)gps_freq;

  State X_simulation = State::Identity(),
        X_unfiltered = State::Identity(); // propagation only, for comparison purposes

  // Define a control vector and its noise and covariance
  Control      u_simu, u_est, u_unfilt;

  Vector6d     u_nom, u_noisy, u_noise;
  Array6d      u_sigmas;
  Matrix6d     U;

  u_nom    << 0.1, 0.0, 0.05, 0.0, 0, 0.05;
  u_sigmas.head<3>().setConstant(std::sqrt(var_odometry));
  u_sigmas.tail<3>().setConstant(std::sqrt(var_gyro));
  U        = (u_sigmas * u_sigmas * 1./dt).matrix().asDiagonal();

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

  // Define the gps measurements
  Eigen::Vector3d y_gps, y_gps_noise;
  Eigen::Array3d  y_gps_sigmas;
  Eigen::Matrix3d R_gps;

  y_gps_sigmas << std::sqrt(6e-3), std::sqrt(6e-3), std::sqrt(6e-3);
  R_gps        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  SystemModel system_model;
  system_model.setCovariance(U);

  StateCovariance state_cov_init = StateCovariance::Identity();
  state_cov_init.bottomRightCorner<3,3>() *= MANIF_PI_4;

  Vector6d n = randn<Array6d>();
  Vector6d X_init_noise = state_cov_init.cwiseSqrt() * n;
  State X_init = X_simulation + State::Tangent(X_init_noise);

  EKF ekf;
  ekf.setState(X_init);
  ekf.setCovariance(state_cov_init);

  SEKF sekf(X_init, state_cov_init);

  IEKF iekf(X_init, state_cov_init);

  UKFM ukfm(X_init, state_cov_init);

  ERTS erts(X_init, state_cov_init);

  SERTS serts(X_init, state_cov_init);

  IERTS ierts(X_init, state_cov_init);

  URTSM urtsm(X_init, state_cov_init);

  // Store some data for plots
  DemoDataCollector<State> collector;
  collector.reserve(
    eot/dt, "UNFI", "EKF", "SEKF", "IEKF", "UKFM", "ERTS", "SERTS", "IERTS", "URTSM"
  );

  // Make T steps. Measure up to K landmarks each time.
  for (double t = 0; t < eot; t += dt) {
    //// I. Simulation

    /// simulate noise
    u_noise = randn<Array6d>(u_sigmas / sqrtdt); // control noise
    u_noisy = u_nom + u_noise;                   // noisy control

    u_simu   = u_nom   * dt;
    u_est    = u_noisy * dt;
    u_unfilt = u_noisy * dt;

    /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X_simulation = system_model(X_simulation, u_simu);

    /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
    for (std::size_t i = 0; i < measurement_models.size(); ++i)  {
      auto measurement_model = measurement_models[i];

      y = measurement_model(X_simulation);      // landmark measurement, before adding noise

      /// simulate noise
      y_noise = randn(y_sigmas);                // measurement noise

      y = y + y_noise;                          // landmark measurement, noisy
      measurements[i] = y;                      // store for the estimator just below
    }

    //// II. Estimation

    /// First we move

    ekf.propagate(system_model, u_est);

    sekf.propagate(system_model, u_est);

    iekf.propagate(system_model, u_est, dt);

    ukfm.propagate(system_model, u_est);

    erts.propagate(system_model, u_est);

    serts.propagate(system_model, u_est);

    ierts.propagate(system_model, u_est, dt);

    urtsm.propagate(system_model, u_est);

    X_unfiltered = system_model(X_unfiltered, u_unfilt);

    /// Then we correct using the measurements of each lmk

    // if (int(t*100) % int(100./landmark_freq) == 0) {
      for (std::size_t i = 0; i < measurement_models.size(); ++i)  {
        // landmark
        auto measurement_model = measurement_models[i];

        // measurement
        y = measurements[i];

        // filter update
        ekf.update(measurement_model, y);

        sekf.update(measurement_model, y);

        iekf.update(measurement_model, y);

        ukfm.update(measurement_model, y);

        erts.update(measurement_model, y);

        serts.update(measurement_model, y);

        ierts.update(measurement_model, y);

        urtsm.update(measurement_model, y);
      }
    // }

    // if (int(t*100) % int(100./gps_freq) == 0) {

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

      erts.update(gps_measurement_model, y_gps);

      serts.update(gps_measurement_model, y_gps);

      ierts.update(gps_measurement_model, y_gps);

      urtsm.update(gps_measurement_model, y_gps);
    // }

    //// III. Results

    collector.collect(X_simulation, t);

    collector.collect("UNFI", X_unfiltered, StateCovariance::Zero(), t);

    collector.collect("EKF",  ekf.getState(), ekf.getCovariance(), t);
    collector.collect("SEKF", sekf.getState(), sekf.getCovariance(), t);
    collector.collect("IEKF", iekf.getState(), iekf.getCovariance(), t);
    collector.collect("UKFM", ukfm.getState(), ukfm.getCovariance(), t);
  }

  // END OF TEMPORAL LOOP. DONE.

  // Batch backward pass - smoothing
  {
    erts.smooth();
    const auto& Xs_erts = erts.getStates();
    const auto& Ps_erts = erts.getCovariances();

    serts.smooth();
    const auto& Xs_serts = serts.getStates();
    const auto& Ps_serts = serts.getCovariances();

    ierts.smooth();
    const auto& Xs_ierts = ierts.getStates();
    const auto& Ps_ierts = ierts.getCovariances();

    urtsm.smooth();
    const auto& Xs_urtsm = urtsm.getStates();
    const auto& Ps_urtsm = urtsm.getCovariances();

    // Collect smoothers' data
    double t=0;
    for (std::size_t i=0; i<Xs_erts.size(); ++i, t+=dt) {
      collector.collect("ERTS", Xs_erts[i], Ps_erts[i], t);
      collector.collect("SERTS", Xs_serts[i], Ps_serts[i], t);
      collector.collect("IERTS", Xs_ierts[i], Ps_ierts[i], t);
      collector.collect("URTSM", Xs_urtsm[i], Ps_urtsm[i], t);
    }
  }

  // print the trajectory
  if (!quiet) {
    KALMANIF_DEMO_PRINT_TRAJECTORY(collector);
  }

  // Generate some metrics and print them
  DemoDataProcessor<State>().process(collector).print();

  // Actually plots only if PLOT_EXAMPLES=ON
  DemoTrajPlotter<State>::plot(collector, filename, plot_trajectory);
  DemoDataPlotter<State>::plot(collector, filename, plot_error);

  return EXIT_SUCCESS;
}
