#ifndef _KALMANIF_KALMANIF_SQUARE_ROOT_EXTENDED_KALMAN_IMPL_FILTER_H_
#define _KALMANIF_KALMANIF_SQUARE_ROOT_EXTENDED_KALMAN_IMPL_FILTER_H_

namespace kalmanif {

// Forward declaration
template <typename Derived> struct SystemModelBase;
template <typename Filter> struct RauchTungStriebelSmoother;

template <typename StateType>
struct SquareRootExtendedKalmanFilter
  : public internal::KalmanFilterBase<
      SquareRootExtendedKalmanFilter<StateType>
    >
  , public internal::CovarianceSquareRootBase<StateType> {

  using Base = internal::KalmanFilterBase<
    SquareRootExtendedKalmanFilter<StateType>
  >;
  using CovarianceSqrtBase = internal::CovarianceSquareRootBase<StateType>;

  using typename Base::Scalar;
  using typename Base::State;
  using Base::setState;
  using Base::getState;
  using CovarianceSqrtBase::setCovariance;
  using CovarianceSqrtBase::getCovariance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  KALMANIF_DEFAULT_CONSTRUCTOR(SquareRootExtendedKalmanFilter);

  SquareRootExtendedKalmanFilter(
    const State& state_init,
    const Eigen::Ref<const Covariance<State>>& cov_init
  ) {
    setState(state_init);
    setCovariance(cov_init);
  }

protected:

  using Base::x;
  using CovarianceSqrtBase::S;

  friend Base;
  friend RauchTungStriebelSmoother<SquareRootExtendedKalmanFilter<StateType>>;

  Jacobian<State, State> A_ = Jacobian<State, State>::Zero();

  const Jacobian<State, State>& getA() const {
    return A_;
  }

  template <class SystemModelDerived, typename... Args>
  const State& propagate_impl(
    const Linearized<SystemModelBase<SystemModelDerived>>& f,
    const typename internal::traits<SystemModelDerived>::Control& u,
    Args&&... args
  ) {
    using Control = typename internal::traits<SystemModelDerived>::Control;

    //! System model jacobian
    Jacobian<State, State> F;

    //! System model noise jacobian
    Jacobian<State, Control> W;

    // propagate state
    x = f(x, u, F, W, std::forward<Args>(args)...);

    // propagate covariance
    computePropagatedCovarianceSquareRoot<State, Control>(
      F, S, W, f.getCovarianceSquareRoot(), S
    );

    A_ = F.transpose();

    KALMANIF_ASSERT(
      isCovariance(getCovariance()),
      "SEKF::propagate: Updated matrix P is not a covariance."
    );

    // return state propagateion
    return getState();
  }

  /**
   * @brief Perform filter update step using measurement \f$z\f$
   * and corresponding measurement model
   *
   * @param [in] m The Measurement model
   * @param [in] z The measurement vector
   * @return The updated state estimate
   */
  template <class MeasurementModelDerived>
  const State& update_impl(
    const Linearized<MeasurementModelBase<MeasurementModelDerived>>& h,
    const typename internal::traits<MeasurementModelDerived>::Measurement& y
  ) {
    using Measurement =
      typename internal::traits<MeasurementModelDerived>::Measurement;

    Jacobian<Measurement, State> H;
    Jacobian<Measurement, Measurement> V;

    // compute expectation
    Measurement e = h(x, H, V);

    // compute innovation covariance
    CovarianceSquareRoot<Measurement> S_y;
    computePropagatedCovarianceSquareRoot<Measurement, Measurement>(
      H, S, V, h.getCovarianceSquareRoot(), S_y
    );

    // compute kalman gain, solve using backsubstitution
    // AX=B with B = HSS^T and X = K^T and A = S_yS_y^T
    KalmanGain<State, Measurement> K = S_y.solve(
      H * S.reconstructedMatrix()
    ).transpose();

    // Update state using computed kalman gain and innovation
    // @todo Fix
    x += typename State::Tangent(K * (y - e));

    // Update covariance
    // TODO: update covariance without using decomposition
    SquareMatrix<Scalar, internal::traits<State>::Size> P = S.reconstructedMatrix();
    S.compute(P - K * H * P);

    KALMANIF_ASSERT(
      S.info() == Eigen::Success,
      "Failed to compute updated square root matrix"
    );

    KALMANIF_ASSERT(
      isCovariance(getCovariance()),
      "SEKF::update: Updated matrix P is not a covariance."
    );

    // return updated state estimate
    return getState();
  }

  /**
   * @brief Compute the propagateed state or
   * innovation covariance (as square root).
   *
   * The formula for computing the propagated square root covariance matrix
   * can be deduced in a very straight forward way using the method outlined in the
   * [Unscented Kalman Filter Tutorial](https://cse.sc.edu/~terejanu/files/tutorialUKF.pdf)
   * by Gabriel A. Terejanu for the UKF (Section 3, formulas (27) and (28)).
   *
   * Starting from the standard update formula
   *
   *     \f[ \hat{P} = FPF^T + WQW^T \f]
   *
   * and using the square-root decomposition
   * \f$ P = SS^T \f$ with \f$S\f$ being lower-triagonal
   * as well as the (lower triangular) square root
   * \f$\sqrt{Q}\f$ of \f$Q\f$ this can be formulated as
   *
   *     \f{align*}{
   *         \hat{P}  &= FSS^TF^T + W\sqrt{Q}\sqrt{Q}^TW^T \\
   *                  &= FS(FS)^T + W\sqrt{Q}(W\sqrt{Q})^T \\
   *                  &=  \begin{bmatrix} FS & W\sqrt{Q} \end{bmatrix}
   *                      \begin{bmatrix} (FS)^T \\ (W\sqrt{Q})^T \end{bmatrix}
   *     \f}
   *
   * The blockmatrix
   *
   * \f[ \begin{bmatrix}
   *      (FS)^T \\
   *      (W\sqrt{Q})^T
   *    \end{bmatrix} \in \mathbb{R}^{2n \times n} \f]
   *
   * can then be decomposed into a product of matrices
   * \f$OR\f$ with \f$O\f$ being orthogonal and
   * \f$R\f$ being upper triangular
   * (also known as QR decompositon).
   * Using this \f$\hat{P}\f$ can be written as
   *
   *   \f{align*}{
   *       \hat{P}  &=  \begin{bmatrix} FS & W\sqrt{Q} \end{bmatrix}
   *                    \begin{bmatrix} (FS)^T \\ (W\sqrt{Q})^T \end{bmatrix} \\
   *                &= (OR)^TOR \\
   *                &= R^T \underbrace{O^T O}_{=I}R \\
   *                &= LL^T \qquad \text{ with } L := R^T
   *   \f}
   *
   * Thus the lower-triangular square root of
   * \f$\hat{P}\f$ is equivalent to the transpose
   * of the upper-triangular matrix obtained from QR-decompositon
   * of the augmented block matrix
   *
   *   \f[ \begin{bmatrix}
   *        FS & W\sqrt{Q}
   *       \end{bmatrix}^T
   *       =
   *       \begin{bmatrix}
   *         S^T F^T \\ \sqrt{Q}^T W^T
   *       \end{bmatrix} \in \mathbb{R}^{2n \times n} \f]
   *
   * The same can be applied for the innovation covariance by replacing
   * the jacobians and the noise covariance accordingly.
   *
   * @param [in] A The jacobian of state transition or measurement function w.r.t. state or measurement
   * @param [in] S The state covariance (as square root)
   * @param [in] B The jacobian of state transition or measurement function w.r.t. state or measurement
   * @param [in] R The system model or measurement noise covariance (as square root)
   * @param [out] S_pred The propagateed covariance (as square root)
   * @return True on success, false on failure due to numerical issue
   */
  template <class Type, class TypeOther>
  bool computePropagatedCovarianceSquareRoot(
    const Eigen::Ref<const Jacobian<Type, State>>& A,
    const CovarianceSquareRoot<State>& S,
    const Eigen::Ref<const Jacobian<Type, TypeOther>>& B,
    const CovarianceSquareRoot<TypeOther>& R,
    CovarianceSquareRoot<Type>& S_pred
  ) {
    constexpr auto StateSize = internal::traits<State>::Size;
    constexpr auto TypeSize = internal::traits<Type>::Size;
    constexpr auto TypeOtherSize = internal::traits<TypeOther>::Size;
    using TmpMat = Eigen::Matrix<Scalar, StateSize + TypeOtherSize, TypeSize>;

    // Compute QR decomposition of (transposed) augmented matrix
    TmpMat tmp;
    tmp.template topRows<StateSize>().noalias() = S.matrixU() * A.transpose();
    tmp.template bottomRows<TypeOtherSize>().noalias() = R.matrixU() * B.transpose();

    // TODO: Use ColPivHouseholderQR
    // Use Ref<TmpMat> for inplace decomposition
    Eigen::HouseholderQR<Eigen::Ref<TmpMat>> qr(tmp);

    // Set S_pred matrix as upper triangular square root
    S_pred.setU(qr.matrixQR().template topRightCorner<TypeSize, TypeSize>());

    return true;
  }
};

namespace internal {

template <class StateType>
struct traits<SquareRootExtendedKalmanFilter<StateType>> {
  using State = StateType;
};

} // namespace internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_SQUARE_ROOT_EXTENDED_KALMAN_IMPL_FILTER_H_
