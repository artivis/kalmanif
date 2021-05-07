#ifndef _KALMANIF_KALMANIF_KALMAN_FILTER_BASE_H_
#define _KALMANIF_KALMANIF_KALMAN_FILTER_BASE_H_

namespace kalmanif {
namespace internal {

template <typename _Derived>
struct KalmanFilterBase : crtp<_Derived> {

  using State = typename internal::traits<_Derived>::State;
  using Scalar = typename internal::traits<State>::Scalar;

protected:

  using crtp<_Derived>::derived;

  KALMANIF_DEFAULT_CONSTRUCTOR(KalmanFilterBase);

public:

  /**
   * @brief Perform filter propagateion step using system model
   * and no control input (i.e. \f$ u = 0 \f$)
   * @param [in] s The System model
   * @return The updated state estimate
   */
  // template <class SystemModelDerived>
  // const State& propagate(SystemModelBase<SystemModelDerived>& f) {
  //   using Control = typename internal::traits<SystemModelDerived>::Control;
  //   // propagate state (without control)
  //   return propagate(f, Control::Zero());
  // }

  /**
   * @brief Perform filter propagateion step using
   * control input \f$u\f$ and corresponding system model
   * @param [in] s The System model
   * @param [in] u The Control input vector
   * @return The updated state estimate
   */
  template<class SystemModelDerived, typename... Args>
  const State& propagate(
    const SystemModelBase<SystemModelDerived>& f,
    const typename internal::traits<SystemModelDerived>::Control& u,
    Args&&... args
  ) {
    return derived().propagate_impl(f.derived(), u, std::forward<Args>(args)...);
  }

  /**
   * @brief Perform filter update step using measurement \f$z\f$
   * and corresponding measurement model
   *
   * @param [in] m The Measurement model
   * @param [in] z The measurement vector
   * @return The updated state estimate
   */
  template <class MeasurementModelDerived, typename... Args>
  const State& update(
    const MeasurementModelBase<MeasurementModelDerived>& h,
    const typename internal::traits<MeasurementModelDerived>::Measurement& y,
    Args&&... args
  ) {
    return derived().update_impl(h.derived(), y, std::forward<Args>(args)...);
  }

  /**
   * @brief Initialize state
   * @param state The state of the system
   */
  void setState(const State& state) {
    x = state;
  }

  /**
   * @brief Get current state estimate
   */
  const State& getState() const {
    return x;
  }

protected:

  //! Estimated state
  State x = State::Identity();
};

} // internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_KALMAN_FILTER_BASE_H_
