#ifndef _KALMANIF_KALMANIF_MEASUREMENT_MODELS_LANDMARK_MEASUREMENT_MODEL_H_
#define _KALMANIF_KALMANIF_MEASUREMENT_MODELS_LANDMARK_MEASUREMENT_MODEL_H_

namespace kalmanif {

template <typename _State, unsigned int Dim>
struct LandmarkMeasurementModel
  : MeasurementModelBase<LandmarkMeasurementModel<_State, Dim>>
  , Linearized<MeasurementModelBase<LandmarkMeasurementModel<_State, Dim>>>
  , LinearizedInvariant<
      MeasurementModelBase<LandmarkMeasurementModel<_State, Dim>>, Invariance::Right
    > {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  static_assert(
    Dim == 2 || Dim == 3,
    "Unknown Landmark Dim."
  );

  using Base = MeasurementModelBase<LandmarkMeasurementModel<_State, Dim>>;
  using Base::setCovariance;
  using Base::getCovariance;
  using Base::getCovarianceSquareRoot;
  using Base::operator ();

  using State = _State;
  using Scalar = typename State::Scalar;
  using Landmark = Eigen::Matrix<Scalar, Dim, 1>;
  using Measurement = Eigen::Matrix<Scalar, Dim, 1>;

  LandmarkMeasurementModel(
    const Landmark& landmark,
    const Eigen::Ref<Covariance<Measurement>>& R
  ) : landmark_(landmark) {
    setCovariance(R);
  }

  Measurement run(const State& x) const {
    return x.inverse().act(landmark_);
  }

  Measurement run_linearized(
    const State& x,
    Eigen::Ref<Jacobian<Measurement, State>> H,
    Eigen::Ref<Jacobian<Measurement, Measurement>> V
  ) const {
    Jacobian<State, State> J_xi_x;
    Jacobian<Measurement, State> J_e_xi;
    Measurement m = x.inverse(J_xi_x).act(landmark_, J_e_xi, V);
    H = J_e_xi * J_xi_x;
    return m;
  }

  Measurement run_linearized_invariant(
    const State& x,
    Eigen::Ref<Jacobian<Measurement, State>> H,
    Eigen::Ref<Jacobian<Measurement, Measurement>> V
  ) const {
    H.setIdentity();
    if constexpr (Dim == 2) {
      H(0, 2) = -landmark_(1);
      H(1, 2) = +landmark_(0);
    } else {
      // This indexing suits SE3 && SE_2_3
      H.template block<3, 3>(0, 3) = skew(-landmark_);
    }

    V = x.rotation();

    return x.inverse().act(landmark_);
  }

  void setLandmark(const Eigen::Ref<Landmark>& landmark) {
    landmark_ = landmark;
  }

  const Landmark& getLandmark() const {
    return landmark_;
  }

protected:

  Landmark landmark_;
};

template <typename Scalar>
using Landmark2DMeasurementModel = LandmarkMeasurementModel<Scalar, 2>;

template <typename Scalar>
using Landmark3DMeasurementModel = LandmarkMeasurementModel<Scalar, 3>;

namespace internal {

template <class StateType, unsigned int Dim>
struct traits<LandmarkMeasurementModel<StateType, Dim>> {
  using State = StateType;
  using Scalar = typename State::Scalar;
  using Measurement = Eigen::Matrix<Scalar, Dim, 1>;
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_MEASUREMENT_MODELS_LANDMARK_MEASUREMENT_MODEL_H_
