#ifndef _KALMANIF_KALMANIF_IMPL_COVARIANCE_SQUARE_ROOT_BASE_H_
#define _KALMANIF_KALMANIF_IMPL_COVARIANCE_SQUARE_ROOT_BASE_H_

namespace kalmanif {
namespace internal {

/**
 * @brief Base class for objects with Covariance as a square root
 *
 * @tparam StateType The state type
 */
template <typename StateType>
struct CovarianceSquareRootBase {

public:

  /**
   * @brief Get the reconstructed covariance matrix
   */
  Covariance<StateType> getCovariance() const {
    return S.reconstructedMatrix();
  }

  /**
   * @brief Set covariance as a covariance matrix
   * @param [in] covariance The input covariance
   * @return true if the covariance is successfully decomposed, false otherwise
   */
  bool setCovariance(const Eigen::Ref<const Covariance<StateType>>& covariance) {
    KALMANIF_ASSERT(
      // cast to actual type, error: ‘Options’ is not a member of Eigen::Ref
      isCovariance(Covariance<StateType>(covariance)),
      "CovarianceSquareRootBase: Not a covariance matrix!"
    );
    S.compute(covariance);
    return S.info() == Eigen::Success;
  }

  /**
   * @brief Get covariance as square root
   */
  const CovarianceSquareRoot<StateType>& getCovarianceSquareRoot() const {
    return S;
  }

  /**
   * @brief Set covariance using square root
   *
   * @param covariance_square_root Lower triangular matrix
   * representing the covariance square root (i.e. P = LLˆT).
   */
  bool setCovarianceSquareRoot(
    const Covariance<StateType>& covariance_square_root
  ) {
    CovarianceSquareRoot<StateType> S;
    S.setL(covariance_square_root);

    KALMANIF_ASSERT(isCovariance(S.reconstructedMatrix()));

    return true;
  }

protected:

  KALMANIF_DEFAULT_CONSTRUCTOR(CovarianceSquareRootBase);

  //! Covariance square root
  CovarianceSquareRoot<StateType> S =
    CovarianceSquareRoot<StateType>::Identity();
};

} // internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_COVARIANCE_SQUARE_ROOT_BASE_H_
