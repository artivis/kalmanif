#ifndef _KALMANIF_KALMANIF_IMPL_INVARIANT_EXTENDED_KALMAN_FILTER_H_
#define _KALMANIF_KALMANIF_IMPL_INVARIANT_EXTENDED_KALMAN_FILTER_H_

namespace kalmanif {

// Forward declaration
template <typename Derived> struct SystemModelBase;

template <typename StateType, Invariance Iv = Invariance::Right>
struct InvariantExtendedKalmanFilter
  : public internal::KalmanFilterBase<
      InvariantExtendedKalmanFilter<StateType, Iv>
    >
  , public internal::CovarianceBase<StateType> {

  static_assert(
    Iv == Invariance::Right,
    "IEKF: Only Right invariance supported at the moment."
  );

  using Base = internal::KalmanFilterBase<
    InvariantExtendedKalmanFilter<StateType, Iv>
  >;
  using CovarianceBase = internal::CovarianceBase<StateType>;

  using typename Base::State;
  using typename Base::Scalar;
  using Base::setState;
  using CovarianceBase::setCovariance;
  using Base::getState;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  KALMANIF_DEFAULT_CONSTRUCTOR(InvariantExtendedKalmanFilter);

  InvariantExtendedKalmanFilter(
    const State& state_init,
    const Eigen::Ref<const Covariance<State>>& cov_init
  ) {
    setState(state_init);
    setCovariance(cov_init);
  }

protected:

  using Base::x;
  using CovarianceBase::P;

  friend Base;

  template <class SystemModelDerived, Invariance SysIv>
  const State& propagate_impl(
    const LinearizedInvariant<SystemModelBase<SystemModelDerived>, SysIv>& f,
    const typename internal::traits<SystemModelDerived>::Control& u,
    Scalar dt = 1
  ) {
    using Control = typename internal::traits<SystemModelDerived>::Control;

    //! System model jacobian
    Jacobian<State, State> F;

    //! System model noise jacobian
    Jacobian<State, Control> W;

    // propagate state
    x = f(x, u, F, W, dt);

    // propagate covariance
    P = F * P * F.transpose();
    P.noalias() += W * f.getCovariance() * W.transpose();

    // enforceCovariance(P);

    KALMANIF_ASSERT(
      isCovariance(P),
      "IEKF::propagate: Updated matrix P is not a covariance."
    );

    return getState();
  }

  /**
   * @brief Perform filter update step using measurement \f$z\f$
   * and corresponding measurement model
   *
   * @param [in] m The Measurement model
   * @param [in] z The measurement vector
   * @return The updated state estimate
   */
  template <typename MeasurementModelDerived, Invariance MeasIv>
  const State& update_impl(
    const LinearizedInvariant<MeasurementModelBase<MeasurementModelDerived>, MeasIv>& h,
    const typename internal::traits<MeasurementModelDerived>::Measurement& y
  ) {
    using Measurement =
      typename internal::traits<MeasurementModelDerived>::Measurement;
    using Tangent = typename State::Tangent;

    Jacobian<Measurement, State> H;
    Jacobian<Measurement, Measurement> M;

    // compute expectation
    Measurement e = h(x, H, M);

    const Covariance<State>& Ptmp = [&]() {
      if constexpr (MeasIv == Invariance::Right) {
        return P;
      } else {
        // Map covariance to Left invariant (from Right thus)
        auto AdXinv = x.inverse().adj();
        return (AdXinv * P * AdXinv.transpose()).eval();
      }
    }();

    Covariance<Measurement> MRMt = M * h.getCovariance() * M.transpose();

    // compute kalman gain
    KalmanGain<State, Measurement> K =
      Ptmp * H.transpose() * (H * Ptmp * H.transpose() + MRMt).inverse();

    // @todo Fix 'z = R * (y - e)'  with X = [R, t].
    // This is the group action on vector!
    // compute correction using computed kalman gain and innovation
    Tangent dx(-(K * (M * (y - e))));

    // Update state using correction
    if constexpr (MeasIv == Invariance::Right) {
      x = dx + x; // Right invariant: Exp(-dx) * x
    } else {
      x = x + dx; // Left invariant: x * Exp(-dx)
    }

    Covariance<State> IKH = Covariance<State>::Identity() - K * H;

    // Update covariance
    // Use the 'Joseph' equation which is numerically more stable
    // P = (I - K.H).P.(I - K.H)^T + K.R.K^T
    if constexpr (MeasIv == Invariance::Right){
      P = IKH * Ptmp * IKH.transpose();
      P.noalias() += K * MRMt * K.transpose();
    } else {
      // Map covariance back to Right invariant (from Left thus)
      auto AdX = x.adj();
      P.noalias() = AdX * (IKH * Ptmp * IKH.transpose() +
                           K * MRMt * K.transpose()      ) * AdX.transpose();
    }

    // enforceCovariance(P);

    KALMANIF_ASSERT(
      isCovariance(P),
      "IEKF::update: Updated matrix P is not a covariance."
    );

    return getState();
  }
};

namespace internal {

template <class StateType, Invariance Iv>
struct traits<InvariantExtendedKalmanFilter<StateType, Iv>> {
  using State = StateType;
};

} // namespace internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_INVARIANT_EXTENDED_KALMAN_FILTER_H_
