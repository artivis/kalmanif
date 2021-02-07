#ifndef _KALMANIF_KALMANIF_IMPL_CHOLESKY_H_
#define _KALMANIF_KALMANIF_IMPL_CHOLESKY_H_

namespace kalmanif {

/**
 * @brief Cholesky square root decomposition
 * of a symmetric positive-definite matrix.
 *
 * @tparam _MatrixType The matrix type
 * @tparam _UpLo Square root form (Eigen::Lower or Eigen::Upper)
 */
template <typename _MatrixType, int _UpLo = Eigen::Lower>
class Cholesky : public Eigen::LLT< _MatrixType, _UpLo > {

public:

  Cholesky() = default;
  ~Cholesky() = default;

  /**
   * @brief Construct cholesky square root decomposition from matrix
   * @param m The matrix to be decomposed
   */
  Cholesky(const _MatrixType& m) : Eigen::LLT<_MatrixType, _UpLo>(m) {}

  /**
   * @brief Set decomposition to identity
   */
  Cholesky& setIdentity() {
    this->m_matrix.setIdentity();
    this->m_isInitialized = true;
    return *this;
  }

  /**
   * @brief Get a new Cholesky object set to identity.
   */
  static Cholesky Identity() {
    const static Cholesky I(Cholesky().setIdentity());
    return I;
  }

  /**
   * @brief Check whether the decomposed matrix is the identity matrix
   */
  bool isIdentity() const {
    eigen_assert(this->m_isInitialized && "LLT is not initialized.");
    return this->m_matrix.isIdentity();
  }

  /**
   * @brief Set lower triangular part of the decomposition
   * @param [in] matrix The lower part stored in a full matrix
   */
  template <typename Derived>
  Cholesky& setL(const Eigen::MatrixBase <Derived>& matrix) {
    this->m_matrix = matrix.template triangularView<Eigen::Lower>();
    this->m_isInitialized = true;
    return *this;
  }

  /**
   * @brief Set upper triangular part of the decomposition
   * @param [in] matrix The upper part stored in a full matrix
   */
  template <typename Derived>
  Cholesky& setU(const Eigen::MatrixBase <Derived>& matrix) {
    this->m_matrix = matrix.template triangularView<Eigen::Upper>().adjoint();
    this->m_isInitialized = true;
    return *this;
  }
};

} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_CHOLESKY_H_
