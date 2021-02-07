#ifndef _KALMANIF_KALMANIF_IMPL_EIGEN_H_
#define _KALMANIF_KALMANIF_IMPL_EIGEN_H_

#include <Eigen/Dense>

namespace kalmanif {

/**
 * @brief Check if the input (square) matrix is symmetric
 *
 * @tparam _EigenDerived The EigenDerived type of the matrix
 * @param M The matrix to test for symmetry
 * @param eps The test tolerance
 * @return true if the matrix is symmetric
 * @return false if the matrix is not symmetric
 */
template <typename _EigenDerived>
bool isSymmetric(
  const Eigen::MatrixBase<_EigenDerived>& M,
  const typename _EigenDerived::Scalar eps = 1e-8
) {
  // @todo assert square
  return M.isApprox(M.transpose(), eps);
}

/**
 * @brief Enforce a (square) matrix to be symmetric
 *
 * @tparam _EigenDerived The EigenDerived type of the matrix
 * @param M The matrix to force symmetry on
 * @param eps The symmetry test tolerance
 * @return true if enforcing symmetry is successful, false otherwise
 */
template <typename _EigenDerived>
bool enforceSymmetric(
  Eigen::MatrixBase<_EigenDerived>& M,
  const typename _EigenDerived::Scalar eps = 1e-8
) {
  // @todo assert square
  using Scalar = typename _EigenDerived::Scalar;
  M = Scalar(0.5) * (M + M.transpose());
  return isSymmetric(M, eps);
}

/**
 * @brief Check if the input matrix is positive definite
 *
 * @tparam _EigenDerived The EigenDerived type of the matrix
 * @param M The matrix to test for positive definite
 * @param eps The test tolerance
 * @return true if the matrix is positive definite
 * @return false if the matrix is not positive definite
 */
template <typename _EigenDerived>
bool isPositiveDefinite(
  const Eigen::MatrixBase<_EigenDerived>& M,
  const typename _EigenDerived::Scalar eps = 1e-8
) {
  Eigen::SelfAdjointEigenSolver<_EigenDerived> eigensolver(M);
  KALMANIF_ASSERT(eigensolver.info() == Eigen::Success);
  if (eigensolver.info() == Eigen::Success) {
    // All eigenvalues must be >= 0:
    return (eigensolver.eigenvalues().array() >= eps).all();
  }
  return false;
}

/**
 * @brief Enforce a matrix to be positive definite
 *
 * @tparam _EigenDerived The EigenDerived type of the matrix
 * @param M The matrix to force for positive definite
 * @param eps The test tolerance
 * @return true if enforcing positive definite is successful, false otherwise
 */
template <typename _EigenDerived>
bool enforcePositiveDefinite(
  Eigen::MatrixBase<_EigenDerived>& M,
  const typename _EigenDerived::Scalar eps = 1e-8
) {
  Eigen::SelfAdjointEigenSolver<_EigenDerived> eigensolver(M);
  KALMANIF_ASSERT(eigensolver.info() == Eigen::Success);

  if (eigensolver.info() == Eigen::Success) {
    // All eigenvalues must be >= 0:
    using Scalar = typename _EigenDerived::Scalar;
    Scalar epsilon = eps;
    while ((eigensolver.eigenvalues().array() < eps).any()) {
      M.noalias() = eigensolver.eigenvectors() *
                    eigensolver.eigenvalues().cwiseMax(epsilon).asDiagonal() *
                    eigensolver.eigenvectors().transpose();
      eigensolver.compute(M);
      epsilon *= Scalar(10);
    }

    KALMANIF_ASSERT(
      isPositiveDefinite(M, eps),
      "Failed to make matrix positive definite."
    );

    return epsilon != eps;
  }

  return false;
}

/**
 * @brief Check if the input matrix is a covariance matrix
 * (symmetric positive definite matrix)
 *
 * @tparam _EigenDerived The EigenDerived type of the matrix
 * @param M The matrix to test for covariance
 * @param eps The test tolerance
 * @return true is the matrix is a covariance, false otherwise
 *
 * @see isSymmetric
 * @see isPositiveDefinite
 */
template <typename _EigenDerived>
bool isCovariance(
  const Eigen::MatrixBase<_EigenDerived>& M,
  const typename _EigenDerived::Scalar eps = 1e-8
) {
  return isSymmetric(M, eps) && isPositiveDefinite(M, eps);
}

/**
 * @brief Enforce a matrix to be a covariance matrix
 * (symmetric positive definite matrix)
 *
 * @tparam _EigenDerived  The EigenDerived type of the matrix
 * @param M The matrix to force as a covariance matrix
 * @param eps The test tolerance
 * @return true if enforcing covariance is successful, false otherwise
 *
 * @see enforceSymmetric
 * @see enforcePositiveDefinite
 */
template <typename _EigenDerived>
bool enforceCovariance(
  Eigen::MatrixBase<_EigenDerived>& M,
  const typename _EigenDerived::Scalar eps = 1e-8
) {
  return enforceSymmetric(M, eps) && enforcePositiveDefinite(M, eps);
}

/**
 * @brief Return a 2x2 skew matrix given a scalar.
 * @note [s] = | 0 -s |
 *             | s  0 |
 * @return A 2x2 skew matrix
 */
template <typename _Scalar>
typename std::enable_if<
  std::is_arithmetic<_Scalar>::value, Eigen::Matrix<_Scalar, 2, 2>
>::type
skew(const _Scalar s)
{
  return (
    Eigen::Matrix<_Scalar, 2, 2>() << _Scalar(0.), -s, s, _Scalar(0.)
  ).finished();
}

/**
 * @brief Return a 3x3 skew matrix given 3-vector.
 * @note [v] = | 0     -v(2) +v(1) |
 *             | +v(2)  0    -v(0) |
 *             | -v(1) +v(0)  0    |
 * @return a 3x3 skew matrix
 */
template <typename _Derived>
typename std::enable_if<
  std::is_base_of_v<Eigen::MatrixBase<_Derived>, _Derived> &&
  _Derived::RowsAtCompileTime == 3,
  Eigen::Matrix<typename _Derived::Scalar, 3, 3>
>::type
skew(const Eigen::MatrixBase<_Derived>& v)
{
  using T = typename _Derived::Scalar;
  return (
    Eigen::Matrix<T, 3, 3>() <<
     T(0.),  -v(2),   +v(1),
    +v(2),    T(0.),  -v(0),
    -v(1),   +v(0),    T(0.)
  ).finished();
}

namespace internal {

template <typename Derived>
void test_eigen_matrix_base(Eigen::MatrixBase<Derived>&& s) {}

template <class, typename T> struct is_eigen_matrix_impl : std::false_type {};
template <typename T> struct
is_eigen_matrix_impl<decltype(test_eigen_matrix_base(std::declval<T>())), T>
  : std::true_type {};
template <typename T> struct is_eigen_matrix
  : is_eigen_matrix_impl<void, T> {};

template <typename T>
using enable_if_is_eigen_matrix =
  typename std::enable_if<is_eigen_matrix<T>::value>::type;

/**
 * @brief traits specialization for Eigen::Matrix
 */
template <typename Matrix>
struct traits<Matrix, enable_if_is_eigen_matrix<Matrix>> {
  using Scalar = typename Matrix::Scalar;
  static constexpr auto Size = Matrix::RowsAtCompileTime;
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_EIGEN_H_
