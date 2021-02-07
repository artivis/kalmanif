#ifndef _KALMANIF_KALMANIF_IMPL_LINEARIZED_INVARIANT_H_
#define _KALMANIF_KALMANIF_IMPL_LINEARIZED_INVARIANT_H_

namespace kalmanif {

/**
 * @brief Base class for linearized invariant models
 *
 * @tparam Derived The linearized invariant model type
 * @tparam Iv The invariance type of the model
 *
 * @see Invariance
 */
template <typename Derived, Invariance Iv> struct LinearizedInvariant;

/**
 * @brief Specialization for linearized invariant system models
 *
 * @tparam Derived The linearized invariant system model
 * @tparam Iv The invariance type of the model
 *
 * @see Invariance
 */
template <typename Derived, Invariance Iv>
struct LinearizedInvariant<SystemModelBase<Derived>, Iv> {
private:

  inline const Derived &derived() const & noexcept {
    return *static_cast<Derived const *>(this);
  }

public:

  static constexpr Invariance ModelInvariance = Iv;

  template <typename... Args>
  auto operator ()(Args&&... args) const {
    return derived().run_linearized_invariant(std::forward<Args>(args)...);
  }

  auto getCovariance() const {
    return derived().getCovariance();
  }
};

/**
 * @brief Specialization for linearized invariant measurement models
 *
 * @tparam Derived The linearized invariant measurement model
 * @tparam Iv The invariance type of the model
 *
 * @see Invariance
 */
template <typename Derived, Invariance Iv>
struct LinearizedInvariant<MeasurementModelBase<Derived>, Iv> {
private:

  inline const Derived &derived() const & noexcept {
    return *static_cast<Derived const *>(this);
  }

public:

  static constexpr Invariance ModelInvariance = Iv;

  template <typename... Args>
  auto operator ()(Args&&... args) const {
    return derived().run_linearized_invariant(std::forward<Args>(args)...);
  }

  auto getCovariance() const {
    return derived().getCovariance();
  }
};

} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_LINEARIZED_INVARIANT_H_