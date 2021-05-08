#ifndef _KALMANIF_KALMANIF_MEASUREMENT_MODELS_DUMMY_GPS_MEASUREMENT_MODEL_H_
#define _KALMANIF_KALMANIF_MEASUREMENT_MODELS_DUMMY_GPS_MEASUREMENT_MODEL_H_

namespace kalmanif {

template <typename _State>
struct DummyGPSMeasurementModel
  : MeasurementModelBase<DummyGPSMeasurementModel<_State>>
  , Linearized<MeasurementModelBase<DummyGPSMeasurementModel<_State>>>
  , LinearizedInvariant<MeasurementModelBase<DummyGPSMeasurementModel<_State>>> {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Base = MeasurementModelBase<DummyGPSMeasurementModel<_State>>;
  using Base::setCovariance;
  using Base::getCovariance;
  using Base::getCovarianceSquareRoot;
  using Base::operator ();

  using State = _State;
  using Scalar = typename State::Scalar;
  static constexpr auto Dim = State::Dim;
  using Measurement = Eigen::Matrix<Scalar, Dim, 1>;

  DummyGPSMeasurementModel(const Eigen::Ref<Covariance<Measurement>>& R) {
    setCovariance(R);
  }

  Measurement run(const State& x) const {
    return x.translation();
  }

  Measurement run_linearized(
    const State& x,
    Eigen::Ref<Jacobian<Measurement, State>> H,
    Eigen::Ref<Jacobian<Measurement, Measurement>> V
  ) const {
    V = x.rotation();
    H.template topLeftCorner<Dim, Dim>() = V;
    H.template topRightCorner<Dim, State::DoF-Dim>().setZero();
    return x.translation();
  }

  Measurement run_linearized_invariant(
    const State& x,
    Eigen::Ref<Jacobian<Measurement, State>> H,
    Eigen::Ref<Jacobian<Measurement, Measurement>> V
  ) const {
    // H.setZero();
    H.template topLeftCorner<Dim, Dim>() = -Eigen::Matrix<Scalar, Dim, Dim>::Identity();
    H.template topRightCorner<Dim, State::DoF-Dim>().setZero();

    V = x.inverse().rotation();

    return x.translation();
  }
};

namespace internal {

template <class StateType>
struct traits<DummyGPSMeasurementModel<StateType>> {
  using State = StateType;
  using Scalar = typename State::Scalar;
  using Measurement = Eigen::Matrix<Scalar, State::Dim, 1>;
  static constexpr Invariance invariance = Invariance::Left;
};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_MEASUREMENT_MODELS_DUMMY_GPS_MEASUREMENT_MODEL_H_
