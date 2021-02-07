#ifndef _KALMANIF_KALMANIF_IMPL_CRTP_H_
#define _KALMANIF_KALMANIF_IMPL_CRTP_H_

namespace kalmanif {
namespace internal {

/**
 * @brief A base class helper for CRTP
 *
 * @tparam Derived The derived class
 */
template <typename Derived>
struct crtp {
protected:
  /** Return reference to this as derived object */
  inline Derived &derived() & noexcept {
    return *static_cast<Derived *>(this);
  }
  /** Return reference to this as derived object */
  inline const Derived &derived() const & noexcept {
    return *static_cast<Derived const *>(this);
  }
  /** Return reference to this as derived object, when this is rvalue */
  inline Derived &&derived() && noexcept {
    return std::move(*static_cast<Derived *>(this));
  }
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_CRTP_H_
