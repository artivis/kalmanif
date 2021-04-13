#ifndef _KALMANIF_KALMANIF_SYSTEM_MODELS_DIFF_DRIVE_SYSTEM_MODEL_H_
#define _KALMANIF_KALMANIF_SYSTEM_MODELS_DIFF_DRIVE_SYSTEM_MODEL_H_

namespace manif {
// Forward declaration
template <typename Scalar> struct SE2;
template <typename, unsigned int> struct Rn;
template <typename Scalar> using R3 = Rn<Scalar, 3>;
template <typename, template <typename> class...> struct Bundle;
} // namespace manif

namespace kalmanif {

/**
 * @brief The Differential Drive model.
 *
 * @tparam Scalar The scalar type.
 */
template <typename Scalar, WithCalibration Calibration = WithCalibration::Disabled>
struct DiffDriveSystemModel final
  : SystemModelBase<DiffDriveSystemModel<Scalar, Calibration>>
  , Linearized<SystemModelBase<DiffDriveSystemModel<Scalar, Calibration>>>
  , LinearizedInvariant<SystemModelBase<DiffDriveSystemModel<Scalar, Calibration>>> {

  //! System model base
  using Base = SystemModelBase<DiffDriveSystemModel<Scalar, Calibration>>;
  using typename Base::State;
  using typename Base::Control;
  using Base::getCovariance;
  using Base::getCovarianceSquareRoot;
  using Base::operator ();

  using Kinematics = Eigen::Matrix<Scalar, 3, 1>;

protected:

  using Tangent = typename State::Tangent;

public:

  DiffDriveSystemModel(const Kinematics& kinematics)
    : k_(kinematics) { }

  ~DiffDriveSystemModel() = default;

  State run(const State& x, const Control& u) const {
    return x + computeTangent(x, u);
  }

  State run_linearized(
    const State& x,
    const Control& u,
    Eigen::Ref<Jacobian<State, State>> F,
    Eigen::Ref<Jacobian<State, Control>> W
  ) const {
    Jacobian<Tangent, Control> J_t_u;
    Tangent omega = computeTangent(x, u, J_t_u);

    Jacobian<State, Tangent> J_xpo_t;
    State x_plus_omega = x.plus(omega, F, J_xpo_t);

    if constexpr (Calibration == WithCalibration::Enabled) {
      using CalibParams = typename State::template Element<1>::DataType;
      const CalibParams& c = x.template element<1>().coeffs();

      Jacobian<Tangent, State> J_t_x = Jacobian<Tangent, State>::Zero();

      const Scalar pcl = u(0) * c(0);
      const Scalar pcr = u(1) * c(1);
      const Scalar ckd = c(2) * k_(2);
      J_t_x(3, 3) =  Scalar(0.5) * pcl;
      J_t_x(3, 4) =  Scalar(0.5) * pcr;
      J_t_x(3, 5) =  Scalar(0);
      J_t_x(5, 3) = -pcl / ckd;
      J_t_x(5, 4) =  pcr / ckd;
      J_t_x(5, 5) = -omega.template element<0>().angle() / c(2);

      F.noalias() += J_xpo_t * J_t_x;
    }

    W.noalias() = J_xpo_t * J_t_u;

    return x_plus_omega;
  }

  State run_linearized_invariant(
    const State& x,
    const Control& u,
    Eigen::Ref<Jacobian<State, State>> F,
    Eigen::Ref<Jacobian<State, Control>> W,
    double /*dt*/
  ) const {
    Jacobian<Tangent, Control> J_t_u;
    Tangent omega = computeTangent(x, u, J_t_u);
    // @todo check those Jacs on paper...
    F.setIdentity(); //*= dt;
    W.noalias() = -x.adj() * J_t_u;

    return x + omega;
  }

protected:

  Tangent computeTangent(const State& x, const Control& u) const {
    if constexpr (Calibration == WithCalibration::Disabled) {
      KALMANIF_UNUSED(x);
      Scalar dl = Scalar(0.5) * (k_(0) * u(0) + k_(1) * u(1));
      Scalar dtheta = Scalar(1) / k_(2) * (k_(0) * u(0) - k_(1) * u(1));

      return {dl, ds_, dtheta};
    } else {
      using CalibParams = typename State::template Element<1>::DataType;
      const CalibParams& c = x.template element<1>().coeffs();
      Scalar dl = Scalar(0.5) * (c(0) * k_(0) * u(0) + c(1) * k_(1) * u(1));
      Scalar dtheta = Scalar(1) / (c(2) * k_(2)) *
        (c(0) * k_(0) * u(0) - c(1) * k_(1) * u(1));

      return {
        typename State::template Element<0>::Tangent(dl, ds_, dtheta),
        State::template Element<1>::Tangent::Zero()
      };
    }
  }

  Tangent computeTangent(
    const State& x, const Control& u, Eigen::Ref<Jacobian<Tangent, Control>> J_t_u
  ) const {
    if constexpr (Calibration == WithCalibration::Disabled) {
      KALMANIF_UNUSED(x);

      J_t_u(0, 0) =  Scalar(0.5) * k_(0);
      J_t_u(0, 1) =  Scalar(0.5) * k_(1);
      J_t_u.row(1).setZero();
      J_t_u(2, 0) = -k_(0) / k_(2);
      J_t_u(2, 1) =  k_(1) / k_(2);
    } else {
      using CalibParams = typename State::template Element<1>::DataType;
      const CalibParams& c = x.template element<1>().coeffs();

      const Scalar ckl = c(0) * k_(0);
      const Scalar ckr = c(1) * k_(1);
      const Scalar ckd = c(2) * k_(2);
      J_t_u.setZero();
      J_t_u(0, 0) =  Scalar(0.5) * ckl;
      J_t_u(0, 1) =  Scalar(0.5) * ckr;
      J_t_u(2, 0) = -ckl / ckd;
      J_t_u(2, 1) =  ckr / ckd;
    }

    return computeTangent(x, u);
  }

  Scalar ds_ = Scalar(1e-3);
  Kinematics k_;
};

namespace internal {

template <class Scalar, WithCalibration Calibration>
struct traits<DiffDriveSystemModel<Scalar, Calibration>> {
  using State = std::conditional_t<
    Calibration == WithCalibration::Disabled,
    manif::SE2<Scalar>,
    manif::Bundle<Scalar, manif::SE2, manif::R3>
  >;
  using Control = Eigen::Matrix<Scalar, 2, 1>;
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_SYSTEM_MODELS_DIFF_DRIVE_SYSTEM_MODEL_H_
