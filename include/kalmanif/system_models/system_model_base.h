#ifndef _KALMANIF_KALMANIF_SYSTEM_MODELS_SYSTEM_MODEL_BASE_H_
#define _KALMANIF_KALMANIF_SYSTEM_MODELS_SYSTEM_MODEL_BASE_H_

namespace kalmanif {

namespace internal {
// Forward declaration
template <typename _Derived> struct KalmanFilterBase;
} // namespace internal

/**
 * @brief Abstract base class of all system models
 *
 * @param StateType The vector-type of the system state (usually some type derived from kalman::Vector)
 * @param ControlType The vector-type of the control input (usually some type derived from kalman::Vector)
 * @param CovarianceBase The class template used for covariance storage (must be either StandardBase or SquareRootBase)
 */
template <typename _Derived>
struct SystemModelBase
  : internal::crtp<_Derived>
  , public internal::CovarianceBase<
      // @todo Control ??
      typename internal::traits<_Derived>::Control
    > {

protected:

  using Base = internal::CovarianceBase<
    typename internal::traits<_Derived>::Control
  >;

  template <typename> friend struct internal::KalmanFilterBase;

  using internal::crtp<_Derived>::derived;
  using Base::Base;

  KALMANIF_DEFAULT_CONSTRUCTOR(SystemModelBase);

public:

  //! System state type
  using State = typename internal::traits<_Derived>::State;

  //! State Scalar type
  using Scalar = typename internal::traits<State>::Scalar;

  //! System control input type
  using Control = typename internal::traits<_Derived>::Control;

  /**
   * @brief State transition function f
   *
   * Computes the propagateed system state in the next timestep given
   * the current state x and the control input u
   *
   * @return The propagated system state
   */
  template <typename... Args>
  State operator ()(Args&&... args) const {
    return derived().run(std::forward<Args>(args)...);
  }
};

} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_SYSTEM_MODELS_SYSTEM_MODEL_BASE_H_
