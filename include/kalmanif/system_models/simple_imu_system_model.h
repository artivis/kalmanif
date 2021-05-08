#ifndef _KALMANIF_KALMANIF_SYSTEM_MODELS_SIMPLE_IMU_SYSTEM_MODEL_H_
#define _KALMANIF_KALMANIF_SYSTEM_MODELS_SIMPLE_IMU_SYSTEM_MODEL_H_

#include <manif/SE_2_3.h>

namespace kalmanif {

/**
 * @brief Abstract base class of all linearized
 * (first order taylor expansion) system models
 *
 * @param Scalar The scalar type
 * @param ControlType The Lie group tangent control input
 */
template <typename Scalar>
struct SimpleImuSystemModel final
  : SystemModelBase<SimpleImuSystemModel<Scalar>>
  , Linearized<SystemModelBase<SimpleImuSystemModel<Scalar>>>
  , LinearizedInvariant<SystemModelBase<SimpleImuSystemModel<Scalar>>> {

  //! System model base
  using Base = SystemModelBase<SimpleImuSystemModel<Scalar>>;
  using typename Base::State;
  using typename Base::Control;
  using Base::getCovariance;
  using Base::getCovarianceSquareRoot;
  using Base::operator ();

  KALMANIF_DEFAULT_CONSTRUCTOR(SimpleImuSystemModel);

protected:

  using Tangent = typename internal::traits<SimpleImuSystemModel<Scalar>>::Tangent;
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
  using Base::P;

public:

  State run(const State& x, const Control& u, const Scalar dt) const {
    Mat3 Rt = x.rotation().transpose();
    Vec3 lin = x.linearVelocity();
    Vec3 acc_k = u.template head<3>() + Rt * gravity;

    Tangent tau;
    tau << dt * Rt * lin + Scalar(0.5) * dt * dt * acc_k,
           dt * u.template tail<3>(),
           dt * acc_k;

    return x + tau;
  }

  State run_linearized(
    const State& x,
    const Control& u,
    Eigen::Ref<Jacobian<State, State>> F,
    Eigen::Ref<Jacobian<State, Control>> W,
    const Scalar dt
  ) const {
    Mat3 Rt = x.rotation().transpose();
    Vec3 lin = x.linearVelocity();
    Vec3 Rtg = Rt * gravity;
    Vec3 acc_k = u.template head<3>() + Rtg;

    Scalar dt22 = Scalar(0.5) * dt * dt;

    Vec3 accLin = dt * Rt * lin + dt22 * acc_k;

    Tangent tau;
    tau << accLin,
           dt * u.template tail<3>(),
           dt * acc_k;

    Jacobian<Tangent, Control> J_tau_u = Jacobian<Tangent, Control>::Zero();

    J_tau_u(0, 0) = dt;
    J_tau_u(1, 1) = dt;
    J_tau_u(2, 2) = dt;

    J_tau_u(3, 3) = dt;
    J_tau_u(4, 4) = dt;
    J_tau_u(5, 5) = dt;

    J_tau_u(6, 0) = dt22;
    J_tau_u(7, 1) = dt22;
    J_tau_u(8, 2) = dt22;

    Jacobian<State, State> J_xnew_x, J_xnew_tau;
    State x_plus_u = x.plus(tau, J_xnew_x, J_xnew_tau);

    Jacobian<State, State> J_u_x = Jacobian<State, State>::Zero();
    J_u_x.template block<3, 3>(0, 3) = skew(accLin);
    J_u_x(0, 6) = dt;
    J_u_x(1, 7) = dt;
    J_u_x(2, 8) = dt;
    J_u_x.template block<3, 3>(6, 3) = skew(Rtg * dt);

    F.noalias() = J_xnew_tau * J_u_x;
    F.noalias() += J_xnew_x;

    W.noalias() = J_xnew_tau * J_tau_u;

    return x_plus_u;
  }

  State run_linearized_invariant(
    const State& x,
    const Control& u,
    Eigen::Ref<Jacobian<State, State>> F,
    Eigen::Ref<Jacobian<State, Control>> W,
    const Scalar dt
  ) const {

    using std::sqrt;

    // Continuous
    //
    // Fc = | 0   0  I |
    //      | 0   0  0 |
    //      | 0 [g]x 0 |
    //
    // Discrete
    //
    // F ~= exp(Fc dt)
    //    = I + Fc * dt

    F.setIdentity();
    F.template block<3, 3>(0, 6).setIdentity() *= dt;
    F.template block<3, 3>(6, 3) = dt * skew(gravity);

    // Continuous
    //
    // Wc = adj(X)
    //
    // Discrete
    //
    // Q ~= F * Wc * cov(w) * Wc^T * F^T * dt
    // W  = F * Wc * sqrt(dt)

    Mat3 R = x.rotation();
    const Vec3 velocity = x.linearVelocity();

    W.template block<3, 3>(0, 0) = R;
    W.template block<3, 3>(0, 3).noalias() = skew(x.translation()) * R;
    W.template block<3, 3>(3, 0).setZero();
    W.template block<3, 3>(3, 3) = R;
    W.template block<3, 3>(6, 0).setZero();
    W.template block<3, 3>(6, 3).noalias() = skew(velocity) * R;
    W = sqrt(dt) * F * W;

    R.transposeInPlace();

    const Vec3 acc_k = R * gravity + u.template head<3>();

    Tangent tau;
    tau << dt * R * velocity + Scalar(0.5) * dt * dt * acc_k,
           dt * u.template tail<3>(),
           dt * acc_k;

    return x + tau;
  }

protected:

  const Vec3 gravity = Vec3(0, 0, -9.80665);
};

namespace internal {

template <typename Scalar>
struct traits<SimpleImuSystemModel<Scalar>> {
  using State = manif::SE_2_3<Scalar>;
  using Tangent = manif::SE_2_3Tangent<Scalar>;
  using Control = Eigen::Matrix<Scalar, 6, 1>;
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_SYSTEM_MODELS_SIMPLE_IMU_SYSTEM_MODEL_H_
