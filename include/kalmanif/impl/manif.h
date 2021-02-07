#ifndef _KALMANIF_KALMANIF_IMPL_MANIF_H_
#define _KALMANIF_KALMANIF_IMPL_MANIF_H_

namespace kalmanif {
namespace internal {

template <typename Derived>
void test_tangent_base(manif::TangentBase<Derived>&& s) {}

template <class, typename T> struct is_manif_tangent_impl : std::false_type {};
template <typename T> struct
is_manif_tangent_impl<decltype(test_tangent_base(std::declval<T>())), T>
  : std::true_type {};
template <typename T> struct is_manif_tangent
  : is_manif_tangent_impl<void, T> {};

template <typename T>
using enable_if_is_manif_tangent =
  typename std::enable_if<is_manif_tangent<T>::value>::type;

template <typename Tangent>
struct traits<Tangent, enable_if_is_manif_tangent<Tangent>> {
  using Scalar = typename Tangent::Scalar;
  static constexpr auto Size = Tangent::DoF;
};

template <typename Derived>
void test_lie_group_base(manif::LieGroupBase<Derived>&& s) {}

template <class, typename T> struct is_manif_group_impl : std::false_type {};
template <typename T> struct
is_manif_group_impl<decltype(test_lie_group_base(std::declval<T>())), T>
  : std::true_type {};
template <typename T> struct is_manif_group
  : is_manif_group_impl<void, T> {};

template <typename T>
using enable_if_is_manif_lie_group =
  typename std::enable_if<is_manif_group<T>::value>::type;

/**
 * @brief traits specialization for manif::LieGroup objects
 *
 * @tparam LieGroup The manif::LieGroup object
 */
template <typename LieGroup>
struct traits<LieGroup, enable_if_is_manif_lie_group<LieGroup>> {
  using Scalar = typename LieGroup::Scalar;
  static constexpr auto Size = LieGroup::DoF;
};

} // namespace internal
} // namespace manif

#endif // _KALMANIF_KALMANIF_IMPL_MANIF_H_
