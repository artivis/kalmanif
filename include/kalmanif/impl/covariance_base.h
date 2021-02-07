#ifndef _KALMANIF_KALMANIF_IMPL_COVARIANCE_BASE_H_
#define _KALMANIF_KALMANIF_IMPL_COVARIANCE_BASE_H_

namespace kalmanif {
namespace internal {

/**
 * @brief Base class for objects with Covariance
 *
 * @tparam StateType The state type
 */
template <typename StateType>
struct CovarianceBase {
public:

  /**
   * @brief Get covariance
   */
  const Covariance<StateType>& getCovariance() const {
    return P;
  }

  /**
   * @brief Set the covariance
   * @param [in] covariance The input covariance
   *
   * @todo Can't use Eigen::Ref here.
   * error: ‘Options’ is not a member of Eigen::Ref
   */
  bool setCovariance(const Eigen::Ref<const Covariance<StateType>>& covariance) {
    KALMANIF_ASSERT(
      // cast to actual type, error: ‘Options’ is not a member of Eigen::Ref
      isCovariance(Covariance<StateType>(covariance)),
      "CovarianceBase: Not a covariance matrix!"
    );
    P = covariance;
    return true;
  }

  /**
   * @brief Get covariance (as square root)
   */
  CovarianceSquareRoot<StateType> getCovarianceSquareRoot() const {
    return CovarianceSquareRoot<StateType>(P);
  }

  /**
   * @brief Set Covariance using Square Root
   *
   * @param [in] covariance_square_root Lower triangular Matrix
   * representing the covariance square root (i.e. P = LLˆT).
   */
  bool setCovarianceSquareRoot(
    const Covariance<StateType>& covariance_square_root
  ) {
    CovarianceSquareRoot<StateType> S;
    S.setL(covariance_square_root);
    return setCovariance(S.reconstructedMatrix());
  }

protected:

  KALMANIF_DEFAULT_CONSTRUCTOR(CovarianceBase);
  CovarianceBase(const Eigen::Ref<const Covariance<StateType>>& covariance)
    : P(covariance) {}

  //! Covariance
  Covariance<StateType> P = Covariance<StateType>::Identity() * 1e3;
};

} // internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_COVARIANCE_BASE_H_
