/**
 * \file demo_se_2_3.cpp
 *
 *  Created on: Feb 5, 2021
 *     \author: artivis
 *
 *  ---------------------------------------------------------
 *  This file is:
 *  (c) 2021 artivis
 *
 *  adapted from the file se_2_3_localization.cpp in manif:
 *  (c) 2020 Prashanth Ramadoss @ DIC-IIT, Genova, Italy
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
 *  3D Robot localization and linear velocity estimation
 *  based on strap-down IMU model and fixed beacons.
 *
 *  ---------------------------------------------------------
 *
 *  We consider a robot in 3D space surrounded by a small
 *  number of punctual landmarks or _beacons_.
 *  The robot is assumed to be mounted with an IMU whose
 *  measurements are fed as exogeneous inputs to the system.
 *  The robot is able to measure the location
 *  of the beacons w.r.t its own reference frame.
 *  We assume in this example that the IMU frame coincides with the robot frame.
 *
 *  The robot extended pose X is in SE_2(3) and the beacon positions b_k in R^3,
 *
 *      X = |    R   p  v|         // position, orientation and linear velocity
 *          |        1   |
 *          |           1|
 *
 *      b_k = (bx_k, by_k, bz_k)   // lmk coordinates in world frame
 *
 *      alpha_k = (alphax_k, alphay_k, alphaz_k) // linear accelerometer measurements in IMU frame
 *
 *      omega_k = (omegax_k, omegay_k, omegaz_k) // gyroscope measurements in IMU frame
 *
 *      g = (0, 0, -9.80665)  // acceleration due to gravity in world frame
 *
 * Consider robot coordinate frame B and world coordinate frame A.
 * - p is the position of the origin of the robot frame B with respect to the world frame A
 * - R is the orientation of the robot frame B with respect to the world frame A
 * - v is the velocity of the robot frame with respect to the world frame,
 *            expressed in a frame whose origin coincides with the robot frame, oriented similar to the world frame
 *            (it is equivalent to p_dot in continuous time. This is usually called mixed-frame representation
 *            and is denoted as (B[A] v_AB), where B[A] is the mixed frame as described above.
 *            For reference, please see "Multibody Dynamics Notation" by Silvio Traversaro and Alessandro Saccon.
 *            Link: https://research.tue.nl/en/publications/multibody-dynamics-notation-version-2)
 * - a is the frame acceleration in mixed-representation (equivalent to p_doubledot in continuous time).
 * - omega_b as the angular velocity of the robot expressed in the robot frame
 *
 * The kinematic equations (1) can be written as,
 * p <-- p + v dt + 0.5 a dt^2
 * R <-- R Exp_SO3(omega_b)
 * v <-- v + a dt
 *
 * However, we would like to express the kinematics equations in the form,
 * X <-- X * Exp(u)
 * where, X \in SE_2(3), u \in R^9 and u_hat \in se_2(3)
 * Note that here input vector u is expressed in the local frame (robot frame).
 * This can be seen as a motion integration on a manifold defined by the group SE_2(3).
 *
 * The exponential mapping of SE_2(3) is defined as,
 * for u = [u_p, u_w, u_v]
 * Exp(u) = | Exp_SO3(u_w)   JlSO3(u_w) u_p   JlSO3(u_w) u_v |
 *          | 0    0    0                 1                0 |
 *          | 0    0    0                 0                1 |
 * where, JlSO3 is the left Jacobian of the SO(3) group.
 *
 * Please see the Appendix C of the paper "A micro Lie theory for state estimation in robotics",
 * for the definition of the left Jacobian of SO(3).
 * Please see the Appendix D of the paper, for the definition of Exp map for SE(3).
 * The Exp map of SE_2(3) is a simple extension from the Exp map of SE(3).
 * Also, please refer to Example 7 of the paper to understand when and how the left Jacobian of SO(3)
 * appears in the definitions of Exp maps. The Example 7 illustrates the scenario for SE(3).
 * We use a direct extension here for SE_2(3).
 * One can arrive to such a definition by following the convergent Taylor's series expansion
 * for the matrix exponential of the Lie algebra element (Equation 16 of the paper).
 *
 * As a result of X <-- X * Exp(u), we get (2)
 * p <-- p + R JlSO3(u_w) u_p
 * R <-- R Exp_SO3(u_w)
 * v <-- v + R JlSO3(u_w) u_v
 *
 * It is important to notice the subtle difference between (1) and (2) here,
 * which is specifically the influence of the left Jacobian of SO(3) in (2).
 * The approach in (1) considers the motion integration is done by defining
 * the exponential map in R3xSO(3)xR3 instead of SE_2(3),
 * in the sense explored in Example 7 of the Micro Lie theory paper. It must be noted that
 * as dt tends to 0, both sets of equations (1) and (2) tend to be the same, since JlSO3 tends to identity.
 *
 * Since, (2) exploits the algebra of the SE_2(3) group properly,
 * we would like to draw a relationship between the sets of equations (2)
 * and the IMU measurements which will constitute the exogeneous input vector u \in se_2(3).
 *
 * Considering R.T as the transpose of R, the IMU measurements are modeled as,
 *    - linear accelerometer measurements alpha = R.T (a - g) + w_acc
 *    - gyroscope measurements omega = omega_b + w_omega
 * Note that the IMU measurements are expressed in the IMU frame (coincides with the robot frame - assumption).
 * The IMU measurements are corrupted by noise,
 *    - w_omega is the additive white noise affecting the gyroscope measurements
 *    - w_acc is the additive white noise affecting the linear accelerometer measurements
 * It must be noted that we do not consider IMU biases in the IMU measurement model in this example.
 *
 * Taking into account all of the above considerations, the exogenous input vector u (3) becomes,
 *   u = (u_p, u_w, u_v) where,
 *   u_w = omega dt
 *   u_p = (R.T v dt + 0.5 dt^2 (alpha + R.T g)
 *   u_v = (alpha + R.T g) dt
 *
 * This choice of input vector allows us to directly use measurements from the IMU
 * for an unified motion integration involving position, orientation and linear velocity of the robot using SE_2(3).
 * Equations (2) and (3) lead us to the following evolution equations,
 *
 * p <-- p + JlSO3 R.T v dt + 0.5 JlSO3 (alpha + R.T g) dt^2
 * R <-- R Exp_SO3(omega dt)
 * v <-- v + JlSO3 (alpha + R.T g) dt
 *
 * The system propagation noise covariance matrix becomes,
 *    U = diagonal(0, 0, 0, sigma_omegax^2, sigma_omegay^2, sigma_omegaz^2, sigma_accx^2, sigma_accy^2, sigma_accz^2).
 *
 *  At the arrival of a exogeneous input u, the robot pose is updated
 *  with X <-- X * Exp(u) = X + u.
 *
 *  Landmark measurements are of the range and bearing type,
 *  though they are put in Cartesian form for simplicity.
 *  Their noise n is zero mean Gaussian, and is specified
 *  with a covariances matrix R.
 *  We notice that the SE_2(3) action is the same as a
 *  rigid motion action of SE(3).
 *  This is the action of X \in SE_2(3) on a 3-d point b \in R^3 defined as,
 *  X b = R b + p
 *
 *  Thus, the landmark measurements can be expressed as a group action on 3d points,
 *  y = h(X,b) = X^-1 * b
 *
 *      y_k = (brx_k, bry_k, brz_k)    // lmk coordinates in robot frame
 *
 *  We consider the beacons b_k situated at known positions.
 *  We define the extended pose to estimate as X in SE_2(3).
 *  The estimation error dx and its covariance P are expressed
 *  in the tangent space at X.
 *
 *  All these variables are summarized again as follows
 *
 *    X   : robot's extended pose, SE_2(3)
 *    u   : robot control input, u = u(X, y_imu) \in se_2(3) with X as state and y_imu = [alpha, omega] as IMU readings, see Eq. (3)
 *    U   : control perturbation covariance
 *    b_k : k-th landmark position, R^3
 *    y   : Cartesian landmark measurement in robot frame, R^3
 *    R   : covariance of the measurement noise
 *
 *  The motion and measurement models are
 *
 *    X_(t+1) = f(X_t, u) = X_t * Exp ( u )     // motion equation
 *    y_k     = h(X, b_k) = X^-1 * b_k          // measurement equation
 *
 *  The algorithm below comprises first a simulator to
 *  produce measurements, then uses these measurements
 *  to estimate the state, using a Lie-based error-state Kalman filter.
 *
 *  This file has plain code with only one main() function.
 *  There are no function calls other than those involving `manif`.
 *
 *  Printing simulated state and estimated state together
 *  with an unfiltered state (i.e. without Kalman corrections)
 *  allows for evaluating the quality of the estimates.
 */

#include <kalmanif/kalmanif.h>
#include <kalmanif/system_models/simple_imu_system_model.h>

#include "utils/rand.h"
#include "utils/plots.h"
#include "utils/utils.h"

#include <vector>

using namespace kalmanif;
using namespace manif;

using State = SE_2_3d;
using StateCovariance = Covariance<State>;
using SystemModel = SimpleImuSystemModel<State::Scalar>;
using Control = SystemModel::Control;
using MeasurementModel = Landmark3DMeasurementModel<State>;
using Landmark = MeasurementModel::Landmark;
using Measurement = MeasurementModel::Measurement;
using Array6d = Eigen::Array<double, 6, 1>;
using Array9d = Eigen::Array<double, 9, 1>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

using EKF = ExtendedKalmanFilter<State>;
using SEKF = SquareRootExtendedKalmanFilter<State>;
using IEKF = InvariantExtendedKalmanFilter<State>;
using UKFM = UnscentedKalmanFilterManifolds<State>;

int main (int argc, char* argv[]) {

  KALMANIF_DEMO_PROCESS_INPUT(argc, argv);
  KALMANIF_DEMO_PRETTY_PRINT();

  // START CONFIGURATION

  constexpr double dt = 0.01;                 // s
  constexpr int landmark_freq = 50;           // Hz

  // acceleration due to gravity in world frame
  Vector3d g;
  g << 0, 0, -9.80665;

  State X_simulation = State::Identity(),
        X_unfiltered = State::Identity(); // propagation only, for comparison purposes

  // IMU measurements in IMU frame
  Vector3d alpha, alpha_const, omega, alpha_prev, omega_prev;
  alpha_const << 0.1, 0.01, 0.1; // constant acceleration in IMU frame without gravity compensation
  omega << 0.01, 0.1, 0;         // constant angular velocity about x- and y-direction in IMU frame

  // Previous IMU measurements in IMU frame initialized to values expected when stationary
  alpha_prev = alpha = alpha_const - (X_simulation.rotation()).transpose() * g;
  omega_prev << 0, 0, 0;

  // Define a control vector and its noise and covariance
  Control  u_simu, u_est, u_unfilt;

  Vector6d u_nom, u_noisy, u_noise;
  Array6d  u_sigmas;
  Matrix6d U;

  u_sigmas << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
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

  Vector9d n = randn<Array9d>();
  Vector9d X_init_coeffs = state_cov_init.cwiseSqrt() * n;
  State X_init(
    X_init_coeffs(0), X_init_coeffs(1), X_init_coeffs(2),
    X_init_coeffs(3), X_init_coeffs(4), X_init_coeffs(5),
    X_init_coeffs(6), X_init_coeffs(7), X_init_coeffs(8)
  );

  EKF ekf;
  ekf.setState(X_init);
  ekf.setCovariance(state_cov_init);

  SEKF sekf(X_init, state_cov_init);

  IEKF iekf(X_init, state_cov_init);

  UKFM ukfm(X_init, state_cov_init);

  // Store some data for plots
  DemoDataCollector<State> collector;

  // Make 10 steps. Measure up to three landmarks each time.
  for (double t = 0; t < 60; t += dt) {

    //// I. Simulation

    /// input vector
    u_nom << alpha_prev, omega_prev;

    /// simulate noise
    u_noise = randn(u_sigmas);                  // control noise
    u_noisy = u_nom + u_noise;                  // noisy control

    u_simu   = u_nom;
    u_est    = u_noisy;
    u_unfilt = u_noisy;

    /// first we move - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    X_simulation = system_model(X_simulation, u_simu, dt);

    /// update expected IMU measurements
    alpha = alpha_const - X_simulation.rotation().transpose() * g; // update expected IMU measurement after moving

    /// then we measure all landmarks - - - - - - - - - - - - - - - - - - - -
    for (int i = 0; i < measurement_models.size(); ++i) {

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

    X_unfiltered = system_model(X_unfiltered, u_unfilt, dt);

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

    auto X_e = ekf.getState();
    auto X_s = sekf.getState();
    auto X_i = iekf.getState();
    auto X_u = ukfm.getState();

    collector.collect("EKF",  X_simulation, X_e, ekf.getCovariance(), t);
    collector.collect("SEKF", X_simulation, X_s, sekf.getCovariance(), t);
    collector.collect("IEKF", X_simulation, X_i, iekf.getCovariance(), t);
    collector.collect("UKFM", X_simulation, X_u, ukfm.getCovariance(), t);
    collector.collect("UNFI", X_simulation, X_unfiltered, StateCovariance::Zero(), t);

    std::cout << "X simulated      : " << X_simulation.log()                << "\n"
              << "X estimated EKF  : " << X_e.log()
              << " : |d|=" << (X_simulation - X_e).weightedNorm()           << "\n"
              << "X estimated SEKF : " << X_s.log()
              << " : |d|=" << (X_simulation - X_s).weightedNorm()           << "\n"
              << "X estimated IEKF : " << X_i.log()
              << " : |d|=" << (X_simulation - X_i).weightedNorm()           << "\n"
              << "X estimated UKFM : " << X_u.log()
              << " : |d|=" << (X_simulation - X_u).weightedNorm()           << "\n"
              << "X unfilterd      : " << X_unfiltered.log()
              << " : |d|=" << (X_simulation - X_unfiltered).weightedNorm()  << "\n"
              << "----------------------------------"                       << "\n";
  }

  // END OF TEMPORAL LOOP. DONE.

  // Generate some metrics and print them
  DemoDataProcessor<State>().process(collector).print();

  // Actually plots only if PLOT_EXAMPLES=ON
  DemoTrajPlotter<State>::plot(collector, filename, plot_trajectory);
  DemoDataPlotter<State>::plot(collector, filename, plot_error);

  return EXIT_SUCCESS;
}
