#ifndef _KALMANIF_KALMANIF_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BASE_H_
#define _KALMANIF_KALMANIF_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BASE_H_

namespace kalmanif {

namespace internal {
template <typename _Derived> struct KalmanFilterBase;
} // namespace internal

/**
 * @brief Abstract base class of all measurement models
 */
template <typename _Derived>
struct MeasurementModelBase
  : internal::crtp<_Derived>
  , internal::CovarianceBase<typename internal::traits<_Derived>::Measurement> {

protected:

  template <typename> friend struct internal::KalmanFilterBase;

  using internal::crtp<_Derived>::derived;

  KALMANIF_DEFAULT_CONSTRUCTOR(MeasurementModelBase);

public:

  //! System control input type
  using Measurement = typename internal::traits<_Derived>::Measurement;

  /**
   * @brief Measurement model function h
   *
   * propagates the estimated measurement value
   * given the current state estimate x.
   */
  template <typename State, typename... Args>
  Measurement operator ()(State&& x, Args&&... args) const {
    return derived().run(std::forward<State>(x), std::forward<Args>(args)...);
  }
};

} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BASE_H_
