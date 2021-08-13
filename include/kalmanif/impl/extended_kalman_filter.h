#ifndef _KALMANIF_KALMANIF_IMPL_EXTENDED_KALMAN_FILTER_H_
#define _KALMANIF_KALMANIF_IMPL_EXTENDED_KALMAN_FILTER_H_

namespace kalmanif {

// Forward declaration
template <typename Derived> struct SystemModelBase;
template <typename Filter> struct RauchTungStriebelSmoother;

/**
 * @brief The ExtendedKalmanFilter
 *
 * @tparam StateType The state type
 */
template <typename StateType>
struct ExtendedKalmanFilter
  : public internal::KalmanFilterBase<ExtendedKalmanFilter<StateType>>
  , public internal::CovarianceBase<StateType> {

  using Base = internal::KalmanFilterBase<ExtendedKalmanFilter<StateType>>;
  using CovarianceBase = internal::CovarianceBase<StateType>;

  using typename Base::State;
  using Base::setState;
  using CovarianceBase::setCovariance;
  using Base::getState;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  KALMANIF_DEFAULT_CONSTRUCTOR(ExtendedKalmanFilter);

  /**
   * @brief Construct a new Extended Kalman Filter object given
   * an initial state and state covariance
   *
   * @param state_init The initial state
   * @param cov_init The initial state covariance
   */
  ExtendedKalmanFilter(
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
  friend RauchTungStriebelSmoother<ExtendedKalmanFilter<StateType>>;

  Jacobian<State, State> A_ = Jacobian<State, State>::Zero();

  // @todo this definitely is an inelegant workaround
  const Jacobian<State, State>& getA() const {
    return A_;
  }

  /**
   * @brief Perform filter propagation step using the input control \f$u\f$
   * and corresponding system model \f$f\f$
   *
   * @tparam SystemModelDerived The derived system model
   * @tparam Args Variadic list of input arguments for the system model
   * @param [in] f The linearized system model
   * @param [in] u The input control
   * @param [in] args input arguments for the system model
   * @return The propagated state
   */
  template <class SystemModelDerived, typename... Args>
  const State& propagate_impl(
    const Linearized<SystemModelBase<SystemModelDerived>>& f,
    const typename internal::traits<SystemModelDerived>::Control& u,
    Args&&... args
  ) {
    using Control = typename internal::traits<SystemModelDerived>::Control;

    // System model jacobian
    Jacobian<State, State> F;

    // System model noise jacobian
    Jacobian<State, Control> W;

    // propagate state
    x = f(x, u, F, W, std::forward<Args>(args)...);

    // propagate covariance
    P = F * P * F.transpose();
    P.noalias() += W * f.getCovariance() * W.transpose();

    A_ = F.transpose();

    // enforceCovariance(P);

    KALMANIF_ASSERT(
      isCovariance(P),
      "EKF::propagate: Updated matrix P is not a covariance."
    );

    return getState();
  }

  /**
   * @brief Perform filter update step using measurement \f$z\f$
   * and corresponding measurement model
   *
   * @tparam MeasurementModelDerived
   * @param [in] h The linearized measurement model
   * @param [in] y The measurement vector
   * @return The updated state estimate
   */
  template <class MeasurementModelDerived>
  const State& update_impl(
    const Linearized<MeasurementModelBase<MeasurementModelDerived>>& h,
    const typename internal::traits<MeasurementModelDerived>::Measurement& y
  ) {
    using Measurement =
      typename internal::traits<MeasurementModelDerived>::Measurement;

    Jacobian<Measurement, State> H;
    Jacobian<Measurement, Measurement> M;

    // compute expectation
    Measurement e = h(x, H, M);

    Covariance<Measurement> MRMt = M * h.getCovariance() * M.transpose();

    // compute kalman gain
    KalmanGain<State, Measurement> K =
      P * H.transpose() * (H * P * H.transpose() + MRMt).inverse();

    // Update state using computed kalman gain and innovation
    // @todo Fix
    x += typename State::Tangent(K * (y - e));

    Covariance<State> IKH = Covariance<State>::Identity() - K * H;

    // Update covariance
    // Use the 'Joseph' equation which is numerically more stable
    // P = (I - K.H).P.(I - K.H)^T + K.R.K^T
    P = IKH * P * IKH.transpose() + K * MRMt * K.transpose();
    // P -= K * H * P;

    // enforceCovariance(P);

    KALMANIF_ASSERT(
      isCovariance(P),
      "EKF::update: Updated matrix P is not a covariance."
    );

    return getState();
  }
};

namespace internal {

/**
 * @brief traits specialization for ExtendedKalmanFilter
 */
template <class StateType>
struct traits<ExtendedKalmanFilter<StateType>> {
  using State = StateType;
};

} // namespace internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_EXTENDED_KALMAN_FILTER_H_
