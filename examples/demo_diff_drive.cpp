/**
 * \file demo_diff_drive.cpp
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
 *  kalmanif is:
 *  (c) 2015 mherb
 *  (c) 2021 artivis
 *  ---------------------------------------------------------
 *
 *  ---------------------------------------------------------
 *  Demonstration example:
 *
 *  2D differential drive base localization based on fixed beacons.
 *  ---------------------------------------------------------
 *
 *  We consider a differential drive base robot in the plane surrounded
 *  by a small number of punctual landmarks or _beacons_.
 *  The robot receives control actions in the form of
 *  noisy incremental wheel angles (e.g. measured by means of wheel encoders),
 *  and is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *
 *  The robot pose X is in SE(2) and the beacon positions b_k in R^2,
 *
 *          | cos th  -sin th   x |
 *      X = | sin th   cos th   y |  // position and orientation
 *          |   0        0      1 |
 *
 *      b_k = (bx_k, by_k)           // lmk coordinates in world frame
 *
 *  The control signal u is in R^2.
 *
 *      u = (phi_l, phi_r)
 *
 *  Where phi_l and phi_r are respectively the incremental left wheel angle
 *  and the incremental right wheel angle.
 *  The control is corrupted by additive Gaussian noise u_noise,
 *  with covariance
 *
 *    Q = diagonal(sigma_l^2, sigma_r^2).
 *
 *  Assuming constant wheel velocities between times steps,
 *  motion of the vehicle can be described by a small arc of length dl,
 *  angle dtheta and radius dl/dtheta.
 *
 *    dl = 0.5 * (rl * phi_l + rr * phi_r)
 *    dtheta = 1 / dw * (rr * phi_r - rl * phi_l)
 *
 *  where rl and rr are respectively the left wheel radius and the
 *  right wheel radius and dw is the distance between both wheels.
 *
 *  This arc can be expressed in the tangent or velocity space of SE(2),
 *
 *    b = (dl, ds, dtheta)
 *
 *  where ds is a zero-mean perturbation accounting for lateral wheel slippage.
 *
 *  At the arrival of a control u, the robot pose is updated
 *  with X <-- X * Exp(b) = X + b.
 *
 *  Landmark measurements are of the range and bearing type,
 *  though they are put in Cartesian form for simplicity.
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice the rigid motion action y = h(X,b) = X^-1 * b
 *  (see appendix C),
 *
 *      y_k = (brx_k, bry_k)       // lmk coordinates in robot frame
 *
 *  We consider the beacons b_k situated at known positions.
 *  We define the pose to estimate as X in SE(2).
 *  The estimation error dx and its covariance P are expressed
 *  in the tangent space at X.
 *
 *  All these variables are summarized again as follows
 *
 *    X   : robot pose, SE(2)
 *    u   : robot control, (v*dt ; 0 ; w*dt) in se(2)
 *    Q   : control perturbation covariance
 *    b_k : k-th landmark position, R^2
 *    y   : Cartesian landmark measurement in robot frame, R^2
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
 *
 *  Demo partially based on
 *  'Joint on-manifold self-calibration of odometry model and
 *   sensor extrinsics using pre-integration',
 *  J. Deray, J. Sola and J. Andrade-Cetto, ECMR 2019.
 */

#include <kalmanif/kalmanif.h>

#include "utils/rand.h"
#include "utils/plots.h"
#include "utils/utils.h"

#include <manif/SE2.h>
#include <kalmanif/system_models/diff_drive_system_model.h>

#include <vector>

using namespace kalmanif;
using namespace manif;

using Scalar = double;
using SystemModel = DiffDriveSystemModel<Scalar>;
using State = SystemModel::State;
using StateCovariance = Covariance<State>;
using Kinematics = SystemModel::Kinematics;
using Control = SystemModel::Control;
using MeasurementModel = Landmark2DMeasurementModel<State>;
using Landmark = MeasurementModel::Landmark;
using Measurement = MeasurementModel::Measurement;

using EKF = ExtendedKalmanFilter<State>;
using SEKF = SquareRootExtendedKalmanFilter<State>;
using IEKF = InvariantExtendedKalmanFilter<State>;
using UKFM = UnscentedKalmanFilterManifolds<State>;

int main (int argc, char* argv[]) {

  KALMANIF_DEMO_PROCESS_INPUT(argc, argv);
  KALMANIF_DEMO_PRETTY_PRINT();

  // START CONFIGURATION

  constexpr int control_freq = 100;           // Hz
  constexpr double dt = 1./control_freq;      // s
  double sqrtdt = std::sqrt(dt);

  constexpr double var_wheel = 9e-5;          // (rad/s)^2

  constexpr int landmark_freq = 50;           // Hz

  State X_simulation = State::Identity(),
        X_unfiltered = State::Identity(); // propagation only, for comparison purposes

  // Define a control vector and its noise and covariance
  Control             u_simu, u_est, u_unfilt;

  Eigen::Vector2d     u_nom, u_noisy, u_noise;
  Eigen::Array2d      u_sigmas;
  Eigen::Matrix2d     U;

  u_nom    << 0.5, 0.35; // move along an arc
  u_sigmas << std::sqrt(var_wheel), std::sqrt(var_wheel);
  U        = (u_sigmas * u_sigmas).matrix().asDiagonal();

  // Define the beacon's measurements
  Eigen::Vector2d y, y_noise;
  Eigen::Array2d  y_sigmas;
  Eigen::Matrix2d R;

  y_sigmas << 0.01, 0.01;
  R        = (y_sigmas * y_sigmas).matrix().asDiagonal();

  std::vector<MeasurementModel> measurement_models = {
    MeasurementModel(Landmark(2.0,  0.0), R),
    MeasurementModel(Landmark(2.0,  1.0), R),
    MeasurementModel(Landmark(2.0, -1.0), R)
  };

  std::vector<Measurement> measurements(measurement_models.size());

  Kinematics kinematic(0.15, 0.15, 0.4); // wheels radii and separation
  SystemModel system_model(kinematic);
  system_model.setCovariance(U);

  StateCovariance state_cov_init = StateCovariance::Zero();
  state_cov_init(0, 0) = 0.1;
  state_cov_init(1, 1) = 0.1;
  state_cov_init(2, 2) = 0.17;

  Eigen::Vector3d n = randn<Eigen::Array3d>();
  Eigen::Vector3d X_init_coeffs = state_cov_init.cwiseSqrt() * n;
  State X_init(X_init_coeffs(0), X_init_coeffs(1), X_init_coeffs(2));

  EKF ekf;
  ekf.setState(X_init);
  ekf.setCovariance(state_cov_init);

  SEKF sekf(X_init, state_cov_init);

  IEKF iekf(X_init, state_cov_init);

  UKFM ukfm(X_init, state_cov_init);

  // Store some data for plots
  DemoDataCollector<State> collector;

  // Make 10 steps. Measure up to three landmarks each time.
  for (double t = 0; t < 300; t += dt) {
    //// I. Simulation

    /// simulate noise
    u_noise = randn<Eigen::Array2d>(u_sigmas / sqrtdt); // control noise
    u_noisy = u_nom + u_noise;                          // noisy control

    u_simu   = u_nom   * dt;
    u_est    = u_noisy * dt;
    u_unfilt = u_noisy * dt;

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

    X_unfiltered = system_model(X_unfiltered, u_unfilt);

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

    //// III. Results

    auto X_e = ekf.getState();
    auto X_s = sekf.getState();
    auto X_i = iekf.getState();
    auto X_u = ukfm.getState();

    collector.collect("EKF",  X_simulation, X_e, ekf.getCovariance(),  t);
    collector.collect("SEKF", X_simulation, X_s, sekf.getCovariance(), t);
    collector.collect("IEKF", X_simulation, X_i, iekf.getCovariance(), t);
    collector.collect("UKFM", X_simulation, X_u, ukfm.getCovariance(), t);
    collector.collect("UNFI", X_simulation, X_unfiltered, StateCovariance::Zero(), t);

    std::cout << "X simulated      : " << X_simulation            << "\n"
              << "X estimated EKF  : " << X_e
              << " : |d|=" << (X_simulation - X_e).weightedNorm() << "\n"
              << "X estimated SEKF : " << X_s
              << " : |d|=" << (X_simulation - X_s).weightedNorm() << "\n"
              << "X estimated IEKF : " << X_i
              << " : |d|=" << (X_simulation - X_i).weightedNorm() << "\n"
              << "X estimated UKFM : " << X_u
              << " : |d|=" << (X_simulation - X_u).weightedNorm() << "\n"
              << "X unfilterd      : " << X_unfiltered
              << " : |d|=" << (X_simulation - X_unfiltered).weightedNorm() << "\n"
              << "----------------------------------"                      << "\n";
  }

  // END OF TEMPORAL LOOP. DONE.

  // Generate some metrics and print them
  DemoDataProcessor<State>().process(collector).print();

  // Actually plots only if PLOT_EXAMPLES=ON
  DemoTrajPlotter<State>::plot(collector, filename, plot_trajectory);
  DemoDataPlotter<State>::plot(collector, filename, plot_error);

  return EXIT_SUCCESS;
}
