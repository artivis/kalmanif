#ifndef _KALMANIF_KALMANIF_SYSTEM_MODELS_LIE_SYSTEM_MODEL_H_
#define _KALMANIF_KALMANIF_SYSTEM_MODELS_LIE_SYSTEM_MODEL_H_

namespace kalmanif {

/**
 * @brief Abstract base class of all linearized
 * (first order taylor expansion) system models
 *
 * @tparam StateType The Lie group state
 */
template <class StateType>
struct LieSystemModel final
    : SystemModelBase<LieSystemModel<StateType>>
    , Linearized<SystemModelBase<LieSystemModel<StateType>>>
    , LinearizedInvariant<SystemModelBase<LieSystemModel<StateType>>> {

  //! System model base
  using Base = SystemModelBase<LieSystemModel<StateType>>;
  using typename Base::Scalar;
  using typename Base::State;
  using typename Base::Control;
  using Base::getCovariance;
  using Base::getCovarianceSquareRoot;
  using Base::operator ();

  KALMANIF_DEFAULT_CONSTRUCTOR(LieSystemModel);
  LieSystemModel(const Eigen::Ref<const Covariance<StateType>>& covariance)
    : Base(covariance) {}

  State run(const State& x, const Control& u) const {
    return x + u;
  }

  State run_linearized(
    const State& x,
    const Control& u,
    Eigen::Ref<Jacobian<State, State>> F,
    Eigen::Ref<Jacobian<State, Control>> W
  ) const {
    return x.plus(u, F, W);
  }

  template <Invariance Iv = Invariance::Right>
  State run_linearized_invariant(
    const State& x,
    const Control& u,
    Eigen::Ref<Jacobian<State, State>> F,
    Eigen::Ref<Jacobian<State, Control>> W,
    Scalar dt
  ) const {
    if constexpr (Iv == Invariance::Right) {
      F.setIdentity();
      // F *= dt;
      (void)dt;
      W = -(x.adj());
      return x + u;
    } else {
      // @todo double check
      F = (-u).exp().adj();
      W.setIdentity();
      W *= -dt;
      return x + u;
    }
  }
};

namespace internal {

template <class StateType>
struct traits<LieSystemModel<StateType>> {
  using State = StateType;
  using Control = typename State::Tangent;
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_SYSTEM_MODELS_LIE_SYSTEM_MODEL_H_
