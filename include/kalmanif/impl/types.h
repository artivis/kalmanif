#ifndef _KALMANIF_KALMANIF_IMPL_TYPES_H_
#define _KALMANIF_KALMANIF_IMPL_TYPES_H_

namespace kalmanif {

enum class WithCalibration : bool {
  Enabled = true,
  Disabled = false
};

/**
 * @brief An alias for square matrix
 *
 * @tparam T The scalar type
 * @tparam N The square dim
 * @tparam Options See Eigen
 * @tparam MaxRows See Eigen
 * @tparam MaxCols See Eigen
 *
 * @see Eigen::Matrix
 */
template <
  typename T, int N, int Options = 0, int MaxRows = N, int MaxCols = N
>
using SquareMatrix = Eigen::Matrix<T, N, N, Options, MaxRows, MaxCols>;

/**
 * @brief An alias for covariance matrix
 *
 * @tparam Type The vector type for which to generate a covariance
 * (usually a state or measurement type)
 *
 * @see SquareMatrix
 */
template <class Type>
using Covariance = SquareMatrix<
  typename internal::traits<Type>::Scalar,
  internal::traits<Type>::Size
>;

/**
 * @brief An alias for covariance square root matrix
 * @param Type The vector type for which to generate a covariance
 * (usually a state or measurement type)
 *
 * @see Cholesky
 */
template <class Type>
using CovarianceSquareRoot = Cholesky<Covariance<Type>>;

/**
 * @brief An alias for the Kalman Gain matrix
 * @param State The system state type
 * @param Measurement The measurement type
 */
template <
  class State,
  class Measurement,
  int Options = 0,
  int MaxRows = internal::traits<State>::Size,
  int MaxCols = Measurement::RowsAtCompileTime
>
using KalmanGain = Eigen::Matrix<
  typename internal::traits<State>::Scalar,
  internal::traits<State>::Size,
  Measurement::RowsAtCompileTime,
  Options,
  MaxRows,
  MaxCols
>;

/**
 * @brief An alias for the Jacobian matrix J_A_B
 *
 * @tparam A The 'input' vector type (usually a state or measurement type)
 * @tparam B The 'output' vector type (usually a tangent or measurement type)
 * @tparam Options see Eigen
 * @tparam MaxRows see Eigen
 * @tparam MaxCols see Eigen
 *
 * @see Eigen::Matrix
 */
template <
  class A,
  class B,
  int Options = 0,
  int MaxRows = internal::traits<A>::Size,
  int MaxCols = internal::traits<B>::Size
>
using Jacobian = Eigen::Matrix<
  typename internal::traits<A>::Scalar,
  internal::traits<A>::Size,
  internal::traits<B>::Size,
  Options,
  MaxRows,
  MaxCols
>;

} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_TYPES_H_
