#ifndef _KALMANIF_KALMANIF_IMPL_TRAITS_H_
#define _KALMANIF_KALMANIF_IMPL_TRAITS_H_

namespace kalmanif {
namespace internal {

/**
 * @brief The kalmanif traits class.
 *
 * @tparam T The type for which to specialize the traits
 * @tparam Enable a SFINAE argument
 */
template <typename T, class Enable = void> struct traits;

/// @note the following is from the Eigen library
/// here we say once and for all that traits<const T> == traits<T>
///
/// When constness must affect traits, it has to be constness on
/// template parameters on which T itself depends.
/// For example, traits<Map<const T> > != traits<Map<T> >, but
///              traits<const Map<T> > == traits<Map<T> >
template <typename T, class Enable>
struct traits<const T, Enable> : traits<T, Enable> {};

} // namespace internal
} // namespace manif

#endif // _KALMANIF_KALMANIF_IMPL_TRAITS_H_
