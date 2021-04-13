#ifndef _KALMANIF_KALMANIF_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BUNDLE_WRAPPER_H_
#define _KALMANIF_KALMANIF_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BUNDLE_WRAPPER_H_

namespace kalmanif {

template <typename Model, unsigned int Idx = 0>
struct MeasurementModelBundleWrapper
  : MeasurementModelBase<MeasurementModelBundleWrapper<Model, Idx>>
  , Linearized<MeasurementModelBase<MeasurementModelBundleWrapper<Model, Idx>>>
  , LinearizedInvariant<
      MeasurementModelBase<MeasurementModelBundleWrapper<Model, Idx>>
    > {

  static_assert(Idx == 0, "Index 0 only supported at the moment.");

  using Measurement = typename kalmanif::internal::traits<Model>::Measurement;

  using Base = MeasurementModelBase<MeasurementModelBundleWrapper<Model, Idx>>;
  using Base::operator ();

  MeasurementModelBundleWrapper(const Model& model) : model_(model) {}
  MeasurementModelBundleWrapper(Model&& model) : model_(std::move(model)) {}

  template <typename... Args>
  MeasurementModelBundleWrapper(Args&&... args)
    : model_(std::forward<Args>(args)...) {}

  template <typename State, typename... Args>
  Measurement run(const State& x, Args&&... args) const {
    return model_.run(
      x.template element<Idx>(), std::forward<Args>(args)...
    );
  }

  template <typename State, typename... Args>
  Measurement run_linearized(
    const State& x,
    Eigen::Ref<Jacobian<Measurement, State>> H,
    Eigen::Ref<Jacobian<Measurement, Measurement>> V,
    Args&&... args
  ) const {
    constexpr int DoF = State::template Element<Idx>::DoF;
    constexpr int Dim = State::template Element<Idx>::Dim;

    H.setZero();
    V.setZero();
    return model_.run_linearized(
      x.template element<Idx>(),
    //   H.template topLeftCorner<Dim, DoF>(),
    //   V.template topLeftCorner<Dim, Dim>(),
      // @todo check indexing below when the pose is not Element<0> (Idx != 0)
      H.template block<Dim, DoF>(
        std::get<Idx>(manif::internal::traits<State>::DoFIdx),
        std::get<Idx>(manif::internal::traits<State>::DoFIdx)
      ),
      V.template block<Dim, DoF>(
        std::get<Idx>(manif::internal::traits<State>::DoFIdx),
        std::get<Idx>(manif::internal::traits<State>::DoFIdx)
      ),
      std::forward<Args>(args)...
    );
  }

  template <typename State, typename... Args>
  Measurement run_linearized_invariant(
    const State& x,
    Eigen::Ref<Jacobian<Measurement, State>> H,
    Eigen::Ref<Jacobian<Measurement, Measurement>> V,
    Args&&... args
  ) const {
    constexpr int DoF = State::template Element<Idx>::DoF;
    constexpr int Dim = State::template Element<Idx>::Dim;

    H.setZero();
    V.setZero();
    return model_.run_linearized_invariant(
      x.template element<Idx>(),
    //   H.template topLeftCorner<Dim, DoF>(),
    //   V.template topLeftCorner<Dim, Dim>(),
      // @todo check indexing below when the pose is not Element<0> (Idx != 0)
      H.template block<Dim, DoF>(
        std::get<Idx>(manif::internal::traits<State>::DoFIdx),
        std::get<Idx>(manif::internal::traits<State>::DoFIdx)
      ),
      V.template block<Dim, DoF>(
        std::get<Idx>(manif::internal::traits<State>::DoFIdx),
        std::get<Idx>(manif::internal::traits<State>::DoFIdx)
      ),
      std::forward<Args>(args)...
    );
  }

  auto getCovariance() const {
    return model_.getCovariance();
  }

  auto getCovarianceSquareRoot() const {
    return model_.getCovarianceSquareRoot();
  }

protected:

  const Model& model_;
};

namespace internal {

template <class Model, unsigned int Idx>
struct traits<MeasurementModelBundleWrapper<Model, Idx>> : traits<Model> {};

} // namespace internal
} // namespace kalmanif

#endif // _KALMANIF_KALMANIF_MEASUREMENT_MODELS_MEASUREMENT_MODEL_BUNDLE_WRAPPER_H_
