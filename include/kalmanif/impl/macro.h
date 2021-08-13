#ifndef _KALMANIF_KALMANIF_IMPL_MACRO_H_
#define _KALMANIF_KALMANIF_IMPL_MACRO_H_

namespace kalmanif {

/**
 * @brief A namespaced runtime_error for easier catch
 */
struct runtime_error : std::runtime_error {
  using std::runtime_error::runtime_error;
  using std::runtime_error::what;
};

/**
 * @brief A namespaced invalid_argument for easier catch
 */
struct invalid_argument : std::invalid_argument {
  using std::invalid_argument::invalid_argument;
  using std::invalid_argument::what;
};

namespace detail {

template <typename E, typename... Args>
void
#if defined(__GNUC__) || defined(__clang__)
__attribute__(( noinline, cold, noreturn ))
#elif defined(_MSC_VER)
__declspec( noinline, noreturn )
#else
// nothing
#endif
raise(Args&&... args) {
  throw E(std::forward<Args>(args)...);
}

template<typename T> void ignore_unused_variable(const T&) {}

} // namespace detail
} // namespace kalmanif

#define KALMANIF_UNUSED_VARIABLE(x) kalmanif::detail::ignore_unused_variable(x)

#ifdef NDEBUG
# ifndef KALMANIF_NO_DEBUG
#  define KALMANIF_NO_DEBUG
# endif
#endif

// gcc expands __VA_ARGS___ before passing it into the macro.
// Visual Studio expands __VA_ARGS__ after passing it.
// This macro is a workaround to support both
#define __KALMANIF_EXPAND(x) x

#define __KALMANIF_THROW_EXCEPT(msg, except) \
  kalmanif::detail::raise<except>(msg);

#define __KALMANIF_THROW(msg) \
  __KALMANIF_THROW_EXCEPT(msg, kalmanif::runtime_error)

#define __KALMANIF_GET_MACRO_2(_1,_2,NAME,...) NAME

#define KALMANIF_THROW(...)                             \
  __KALMANIF_EXPAND(                                    \
  __KALMANIF_GET_MACRO_2(__VA_ARGS__,                   \
                         __KALMANIF_THROW_EXCEPT,       \
                         __KALMANIF_THROW)(__VA_ARGS__) )

#define __KALMANIF_CHECK_MSG_EXCEPT(cond, msg, except) \
 if (!(cond)) {KALMANIF_THROW(msg, except);}

#define __KALMANIF_CHECK_MSG(cond, msg) \
 __KALMANIF_CHECK_MSG_EXCEPT(cond, msg, kalmanif::runtime_error)

#define __KALMANIF_CHECK(cond)                                    \
 __KALMANIF_CHECK_MSG_EXCEPT(                                     \
   cond, "Condition: '"#cond"' failed!", kalmanif::runtime_error  \
 )

#define __KALMANIF_GET_MACRO_3(_1,_2,_3,NAME,...) NAME

#define KALMANIF_CHECK(...)                             \
 __KALMANIF_EXPAND(                                     \
 __KALMANIF_GET_MACRO_3(__VA_ARGS__,                    \
                        __KALMANIF_CHECK_MSG_EXCEPT,    \
                        __KALMANIF_CHECK_MSG,           \
                        __KALMANIF_CHECK)(__VA_ARGS__) )

// Assertions cost run time and can be turned off.
// You can suppress KALMANIF_ASSERT by defining
// KALMANIF_NO_DEBUG before including manif headers.
// KALMANIF_NO_DEBUG is undefined by default unless NDEBUG is defined.
#ifndef KALMANIF_NO_DEBUG
  #define KALMANIF_ASSERT(...)                            \
    __KALMANIF_EXPAND(                                    \
    __KALMANIF_GET_MACRO_3(__VA_ARGS__,                   \
                           __KALMANIF_CHECK_MSG_EXCEPT,   \
                           __KALMANIF_CHECK_MSG,          \
                           __KALMANIF_CHECK)(__VA_ARGS__) )
#else
  #define KALMANIF_ASSERT(...) ((void)0)
#endif

// Common macros

#define KALMANIF_MAKE_ALIGNED_OPERATOR_NEW_COND                       \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(                                 \
    (Eigen::internal::traits<typename State::DataType>::Alignment>0))
#define KALMANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(X)           \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(                             \
    (Eigen::internal::traits<typename X::DataType>::Alignment>0))

#define KALMANIF_DEFAULT_CONSTRUCTOR(X) \
  X() = default;                        \
  ~X() = default;                       \
  X(const X&) = default;                \
  X(X&&) = default;

  #define KALMANIF_DEFAULT_OPERATOR(X)  \
    X& operator =(X&) = default;        \
    X& operator =(X&&) = default;

#endif // _KALMANIF_KALMANIF_IMPL_MACRO_H_
