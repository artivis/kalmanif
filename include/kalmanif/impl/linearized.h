#ifndef _KALMANIF_KALMANIF_IMPL_LINEARIZED_H_
#define _KALMANIF_KALMANIF_IMPL_LINEARIZED_H_

namespace kalmanif {

/**
 * @brief Base class for linearized models
 *
 * @tparam Derived The linearized model type
 */
template <typename Derived> struct Linearized;

/**
 * @brief Specialization for linearized system models
 *
 * @tparam Derived The linearized system model
 */
template <typename Derived>
struct Linearized<SystemModelBase<Derived>> {
private:

  inline const Derived &derived() const & noexcept {
    return *static_cast<Derived const *>(this);
  }

public:

  template <typename... Args>
  auto operator ()(Args&&... args) const {
    return derived().run_linearized(std::forward<Args>(args)...);
  }

  auto getCovariance() const {
    return derived().getCovariance();
  }

  auto getCovarianceSquareRoot() const {
    return derived().getCovarianceSquareRoot();
  }
};

/**
 * @brief Specialization for linearized measurement models
 *
 * @tparam Derived The linearized measurement model
 */
template <typename Derived>
struct Linearized<MeasurementModelBase<Derived>> {
private:

  inline const Derived &derived() const & noexcept {
    return *static_cast<Derived const *>(this);
  }

public:

  template <typename... Args>
  auto operator ()(Args&&... args) const {
    return derived().run_linearized(std::forward<Args>(args)...);
  }

  auto getCovariance() const {
    return derived().getCovariance();
  }

  auto getCovarianceSquareRoot() const {
    return derived().getCovarianceSquareRoot();
  }
};

} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_LINEARIZED_H_