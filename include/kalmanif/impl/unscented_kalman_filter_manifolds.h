#ifndef _KALMANIF_KALMANIF_IMPL_UNSCENTED_KALMAN_FILTER_MANIFOLDS_H_
#define _KALMANIF_KALMANIF_IMPL_UNSCENTED_KALMAN_FILTER_MANIFOLDS_H_

namespace kalmanif {
namespace {

template <typename Scalar>
struct Weights {
  KALMANIF_DEFAULT_CONSTRUCTOR(Weights);
  KALMANIF_DEFAULT_OPERATOR(Weights);

  Weights(const Scalar l, const Scalar alpha) {
    using std::sqrt;

    const Scalar m = (alpha * alpha - 1) * l;
    const Scalar ml = m + l;

    sqrt_d_lambda = sqrt(ml);
    wj = Scalar(1) / (Scalar(2) * (ml));
    wm = m / (ml);
    w0 = m / (ml) + Scalar(3) - alpha * alpha;
  }

  Scalar sqrt_d_lambda;
  Scalar wj;
  Scalar wm;
  Scalar w0;
};

template <typename Scalar>
std::tuple<Weights<Scalar>, Weights<Scalar>, Weights<Scalar>>
compute_sigma_weights(
  const Scalar state_size,
  const Scalar propagation_noise_size,
  const Scalar alpha_0,
  const Scalar alpha_1,
  const Scalar alpha_2
) {
  KALMANIF_ASSERT(state_size > Scalar(0));
  KALMANIF_ASSERT(propagation_noise_size > Scalar(0));
  KALMANIF_ASSERT(alpha_0 >= Scalar(1e-3) && alpha_0 <= Scalar(1));
  KALMANIF_ASSERT(alpha_1 >= Scalar(1e-3) && alpha_1 <= Scalar(1));
  KALMANIF_ASSERT(alpha_2 >= Scalar(1e-3) && alpha_2 <= Scalar(1));

  return std::make_tuple(
    Weights<Scalar>(state_size, alpha_0),
    Weights<Scalar>(propagation_noise_size, alpha_1),
    Weights<Scalar>(state_size, alpha_2)
  );
}

} // namespace

// Forward declaration
template <typename Derived> struct SystemModelBase;

template <typename StateType, Invariance Iv = Invariance::Right>
struct UnscentedKalmanFilterManifolds
  : public internal::KalmanFilterBase<UnscentedKalmanFilterManifolds<StateType>>
  , public internal::CovarianceBase<StateType> {

  static_assert(
    Iv == Invariance::Right,
    "UKFM: Only Right invariance supported at the moment."
  );

  using Base = internal::KalmanFilterBase<
    UnscentedKalmanFilterManifolds<StateType>
  >;
  using CovarianceBase = internal::CovarianceBase<StateType>;

  using typename Base::Scalar;
  using typename Base::State;
  using Base::setState;
  using CovarianceBase::setCovariance;
  using Base::getState;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  UnscentedKalmanFilterManifolds()
    : Base(), CovarianceBase() {
    std::tie(w_d, w_q, w_u) = compute_sigma_weights<Scalar>(
      internal::traits<State>::Size,
      // @todo fix. This is propagation_noise_size
      internal::traits<State>::Size,
      1e-3,
      1e-3,
      1e-3
    );
  }

  UnscentedKalmanFilterManifolds(
    const State& state_init,
    const Eigen::Ref<const Covariance<State>>& cov_init,
    Scalar alpha0 = 1e-3,
    Scalar alpha1 = 1e-3,
    Scalar alpha2 = 1e-3
  ) : Base(), CovarianceBase() {
    setState(state_init);
    setCovariance(cov_init);

    std::tie(w_d, w_q, w_u) = compute_sigma_weights<Scalar>(
      internal::traits<State>::Size,
      // @todo fix. This is propagation_noise_size
      internal::traits<State>::Size,
      alpha0,
      alpha1,
      alpha2
    );
  }

  ~UnscentedKalmanFilterManifolds() = default;

protected:

  using Base::x;
  using CovarianceBase::P;

  friend Base;

  template <class SystemModelDerived, typename... Args>
  const State& propagate_impl(
    const SystemModelBase<SystemModelDerived>& f,
    const typename internal::traits<SystemModelDerived>::Control& u,
    Args&&... args
  ) {
    using Control = typename internal::traits<SystemModelDerived>::Control;
    using Tangent = typename State::Tangent;
    using MapTangent = Eigen::Map<Tangent>;
    constexpr auto StateSize = internal::traits<State>::Size;
    constexpr auto NoiseSize = internal::traits<Control>::Size;
    using VectorDoF = Eigen::Matrix<Scalar, StateSize, 1>;
    using VectorCoF = Eigen::Matrix<Scalar, NoiseSize, 1>;
    using MatrixDoF = SquareMatrix<Scalar, StateSize>;

    // propagate state
    const State x_new = f(x, u, std::forward<Args>(args)...);

    KALMANIF_ASSERT(
      isPositiveDefinite(P),
      "UKFM::propagate: Matrix P is not positive definite."
    );

    MatrixDoF xis = w_d.sqrt_d_lambda * P.llt().matrixL().toDenseMatrix();

    // sigma points on manifold
    State s_j_p, s_j_m;
    Eigen::Matrix<Scalar, StateSize, StateSize*2> xis_new;
    for (int i = 0; i < StateSize; ++i) {
      if constexpr (Iv == Invariance::Right) {
        s_j_p = MapTangent(xis.col(i).data()) + x;
        s_j_m = Tangent(-xis.col(i)) + x;

        xis_new.col(i) = x_new.lminus(
          f(s_j_p, u, std::forward<Args>(args)...)
        ).coeffs();

        xis_new.col(i + StateSize) = x_new.lminus(
          f(s_j_m, u, std::forward<Args>(args)...)
        ).coeffs();
      } else {
        s_j_p = x + MapTangent(xis.col(i).data());
        s_j_m = x + Tangent(-xis.col(i));

        xis_new.col(i) = x_new.rminus(
          f(s_j_p, u, std::forward<Args>(args)...)
        ).coeffs();

        xis_new.col(i + StateSize) = x_new.rminus(
          f(s_j_m, u, std::forward<Args>(args)...)
        ).coeffs();
      }
    }

    // compute covariance
    VectorDoF xi_mean = w_d.wj * xis_new.rowwise().sum();
    xis_new.colwise() -= xi_mean;

    // sigma points on manifold
    VectorCoF w_p;
    Covariance<Control> Uchol = f.getCovarianceSquareRoot().matrixL();

    Eigen::Matrix<Scalar, StateSize, 2*NoiseSize> xis_new2;
    for (int i = 0; i < NoiseSize; ++i) {
      w_p.noalias() = w_q.sqrt_d_lambda * Uchol.col(i);

      if constexpr (Iv == Invariance::Right) {
        xis_new2.col(i) = x_new.lminus(
          f(x, u + w_p, std::forward<Args>(args)...)
        ).coeffs();

        xis_new2.col(i + NoiseSize) = x_new.lminus(
          f(x, u - w_p, std::forward<Args>(args)...)
        ).coeffs();
      } else {
        xis_new2.col(i) = x_new.rminus(
          f(x, u + w_p, std::forward<Args>(args)...)
        ).coeffs();

        xis_new2.col(i + NoiseSize) = x_new.rminus(
          f(x, u - w_p, std::forward<Args>(args)...)
        ).coeffs();
      }
    }

    VectorDoF xi_mean2 = w_q.wj * xis_new2.rowwise().sum();
    xis_new2.colwise() -= xi_mean2;

    P.noalias() =
      w_d.wj * xis_new * xis_new.transpose()   +  // P new
      w_d.w0 * xi_mean * xi_mean.transpose()   +
      w_q.wj * xis_new2 * xis_new2.transpose() +  // U
      w_q.w0 * xi_mean2 * xi_mean2.transpose();

    enforceCovariance(P);

    KALMANIF_ASSERT(
      isPositiveDefinite(P),
      "UKFM::propagate: Updated matrix P is not positive definite."
    );

    setState(x_new);

    // return propagated state
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
    const MeasurementModelBase<MeasurementModelDerived>& h,
    const typename internal::traits<MeasurementModelDerived>::Measurement& y
  ) {
    using Measurement =
      typename internal::traits<MeasurementModelDerived>::Measurement;
    using Tangent = typename State::Tangent;
    using MapTangent = Eigen::Map<const Tangent>;
    // using MapConstTangent = Eigen::Map<const Tangent>;
    constexpr auto MeasSize = internal::traits<Measurement>::Size;
    constexpr auto DoF = internal::traits<State>::Size;
    using MatrixDoF = SquareMatrix<Scalar, DoF>;

    // compute expectation
    Measurement e = h(x);

    KALMANIF_ASSERT(
      isPositiveDefinite(P),
      "UKFM::update: Matrix P is not positive definite."
    );

    const Covariance<State>& Ptmp = [&]() {
      if constexpr (MeasurementModelDerived::ModelInvariance == Invariance::Right) {
        return P;
      } else {
        // Map covariance to Left invariant (from Right thus)
        auto AdXinv = x.inverse().adj();
        return (AdXinv * P * AdXinv.transpose()).eval();
      }
    }();

    // set sigma points
    MatrixDoF xis = w_u.sqrt_d_lambda * Ptmp.llt().matrixL().toDenseMatrix();

    // compute measurement sigma points
    Eigen::Matrix<Scalar, MeasSize, 2 * DoF> yj;
    for (int i = 0; i < DoF; ++i) {
      if constexpr (MeasurementModelDerived::ModelInvariance == Invariance::Right) {
        yj.col(i) = h( MapTangent(xis.col(i).data()) + x );
        yj.col(i + DoF) = h( Tangent(-xis.col(i)) + x );
      } else {
        yj.col(i) = h( x + MapTangent(xis.col(i).data()) );
        yj.col(i + DoF) = h( x + Tangent(-xis.col(i)) );
      }
    }

    // measurement mean
    Measurement y_bar = w_u.wm * e + w_u.wj * yj.rowwise().sum();

    yj.colwise() -= y_bar;
    e -= y_bar;

    // compute covariance and cross covariance matrices
    SquareMatrix<Scalar, MeasSize> P_yy =
      w_u.w0 * e * e.transpose()   +
      w_u.wj * yj * yj.transpose() +
      h.getCovariance();

    xis.transposeInPlace();
    // @todo replace xis with xij (topRows<DoF>())
    Eigen::Matrix<Scalar, 2 * DoF, DoF> xij;
    xij.template topRows<DoF>() = xis;
    xij.template bottomRows<DoF>() = -xis;

    // Kalman gain
    KalmanGain<State, Measurement> K =
      P_yy.colPivHouseholderQr().solve(w_u.wj * yj * xij).transpose();

    // Update state using computed kalman gain and innovation
    if constexpr (MeasurementModelDerived::ModelInvariance == Invariance::Right) {
      x = Tangent((K * (y - y_bar))) + x;
    } else {
      x = x + Tangent((K * (y - y_bar)));
    }

    // Update covariance
    P.noalias() -= [&]() {
      if constexpr (MeasurementModelDerived::ModelInvariance == Invariance::Right) {
        return (K * P_yy * K.transpose()).eval();
      } else {
        // Map covariance to Left invariant (from Right thus)
        auto AdX = x.adj();
        return (AdX * K * P_yy * K.transpose() * AdX.transpose()).eval();
      }
    }();

    enforceCovariance(P);

    KALMANIF_ASSERT(
      isCovariance(P),
      "UKFM::update: Updated matrix P is not positive definite."
    );

    // return updated state estimate
    return getState();
  }

  //! Unscented transform weights
  Weights<Scalar> w_d, w_q, w_u;
};

namespace internal {

template <class StateType>
struct traits<UnscentedKalmanFilterManifolds<StateType>> {
  using State = StateType;
};

} // namespace internal
} // kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_UNSCENTED_KALMAN_FILTER_MANIFOLDS_H_
