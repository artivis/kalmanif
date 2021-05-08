#ifndef _KALMANIF_KALMANIF_IMPL_RAUCH_TUNG_STRIEBEL_SMOOTHER_H_
#define _KALMANIF_KALMANIF_IMPL_RAUCH_TUNG_STRIEBEL_SMOOTHER_H_

#include <deque>

namespace kalmanif {

// Forward declaration
template <typename Derived> struct SystemModelBase;

namespace internal {

template <typename>
struct is_unscented : std::false_type {};

template <typename T>
struct is_unscented<UnscentedKalmanFilterManifolds<T>> : std::true_type {};

template <typename>
struct is_invariant : std::false_type {};

template <typename T>
struct is_invariant<UnscentedKalmanFilterManifolds<T>> : std::true_type {};

template <typename T>
struct is_invariant<
  InvariantExtendedKalmanFilter<T, Invariance::Right>
> : std::true_type {};

template <typename>
struct is_right_invariant : std::false_type {};

template <typename T>
struct is_right_invariant<UnscentedKalmanFilterManifolds<T>> : std::true_type {};

template <typename T>
struct is_right_invariant<
  InvariantExtendedKalmanFilter<T, Invariance::Right>
> : std::true_type {};

} // namespace internal

/**
 * @brief The Rauch-Tung-Striebel Smoother
 *
 * @note Based on,
 * "The Invariant Rauch-Tung-Striebel Smoother" N. Laan et al. [1]
 * "Bayesian Filtering and Smoothing" S. Särkkä [2]
 *
 * @tparam Filter The underlying filter type.
 */
template <typename Filter>
struct RauchTungStriebelSmoother {

  template <typename T>
  using deque_t = std::deque<T, Eigen::aligned_allocator<T>>;

  using State = typename Filter::State;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  KALMANIF_DEFAULT_CONSTRUCTOR(RauchTungStriebelSmoother);

  RauchTungStriebelSmoother(
    const State& state_init,
    const Eigen::Ref<const Covariance<State>>& cov_init
  ) : filter_(state_init, cov_init) { }

  /**
   * @brief Performs the underlying filter's propagation.
   */
  template <class SystemModelDerived, typename... Args>
  const State& propagate(
    const SystemModelBase<SystemModelDerived>& f,
    const typename internal::traits<SystemModelDerived>::Control& u,
    Args&&... args
  ) {

    const State& xtmp = filter_.propagate(f, u, std::forward<Args>(args)...);
    const Covariance<State> Ptmp = filter_.getCovariance();
    const Jacobian<State, State>& Aktmp = filter_.getA();

    if (updated_) {
      Xfk_pred_.emplace_back(xtmp);
      Pfk_pred_.emplace_back(Ptmp);
      Ak_.emplace_back(Aktmp);
      updated_ = false;
    } else {
      Xfk_pred_.back() = xtmp;
      Pfk_pred_.back() = Ptmp;

      // @todo This does not work
      Ak_.back() = Aktmp * Ak_.back();
    }

    propagated_ = true;

    return filter_.getState();
  }

  /**
   * @brief Performs the underlying filter's update.
   */
  template <typename MeasurementModelDerived, typename... Args>
  const State& update(
    const MeasurementModelBase<MeasurementModelDerived>& h,
    const typename internal::traits<MeasurementModelDerived>::Measurement& y,
    Args&&... args
  ) {

    const State& xtmp = filter_.update(h, y, std::forward<Args>(args)...);
    const Covariance<State>& Ptmp = filter_.getCovariance();

    if (propagated_) {
      Xfk_est_.emplace_back(xtmp);
      Pfk_est_.emplace_back(Ptmp);
      propagated_ = false;
    } else {
      Xfk_est_.back() = xtmp;
      Pfk_est_.back() = Ptmp;
    }

    updated_ = true;

    return filter_.getState();
  }

  /**
   * @brief Run the batch backward pass - the smoothing.
   * @return The smoothed state sequence.
   */
  const deque_t<State>& smooth() {

    Xsk_.clear();
    Psk_.clear();

    if (Xfk_est_.empty())
      return Xsk_;

    // Initialize the smoother
    Xsk_.emplace_back(Xfk_est_.back());
    Psk_.emplace_back(Pfk_est_.back());

    // smoother gain
    Jacobian<State, State> Ks_;

    // Smoothing routine
    for (std::size_t j = Xfk_est_.size() - 1; j > 0; --j) {

      std::size_t k = j - 1;

      // Compute smoother gain
      if constexpr (internal::is_unscented<Filter>{}) {
        Ks_ = Ak_[k] * Pfk_pred_[k].inverse();
      } else {
        Ks_ = Pfk_est_[k] * Ak_[k].transpose() * Pfk_pred_[k+1].inverse();
      }

      // Compute smoothed states and covariances
      if constexpr (!internal::is_invariant<Filter>{}) {
        // See [2]
        Xsk_.emplace_front(
          Xfk_est_[k] + (Ks_ * ( Xsk_.front() - Xfk_pred_[k+1] ))
        );
        Psk_.emplace_front(
          // See [2] Alg. 9.1
          Pfk_est_[k] + Ks_ * ( Psk_.front() - Pfk_pred_[k+1] ) * Ks_.transpose()
      );
      // See [1]
      } else if (internal::is_right_invariant<Filter>{}) {
        Xsk_.emplace_front(
          -(Ks_ * ( Xfk_pred_[k+1].lminus(Xsk_.front()) )) + Xfk_est_[k]
        );
        Psk_.emplace_front(
          Pfk_est_[k] - Ks_ * ( Pfk_pred_[k+1] - Psk_.front() ) * Ks_.transpose() // paper
      );
      } else {
        Xsk_.emplace_front(
          Xfk_est_[k] + (-(Ks_ * ( Xfk_pred_[k+1] - Xsk_.front() )))
        );
        Psk_.emplace_front(
          Pfk_est_[k] - Ks_ * ( Pfk_pred_[k+1] - Psk_.front() ) * Ks_.transpose() // paper
      );
      }
    }

    return Xsk_;
  }

  const State& getState() const {
    return filter_.getState();
  }

  const Covariance<State>& getCovariance() const {
    return filter_.getCovariance();
  }

  const deque_t<State>& getStates() const {
    return Xsk_;
  }

  const deque_t<Covariance<State>>& getCovariances() const {
    return Psk_;
  }

  void clear() {
    Xfk_pred_.clear();
    Xfk_est_.clear();
    Xsk_.clear();
    Pfk_pred_.clear();
    Pfk_est_.clear();
    Psk_.clear();
    Ak_.clear();
  }

protected:

  bool propagated_ = false;
  bool updated_ = true;

  Filter filter_;

  deque_t<State> Xfk_pred_, Xfk_est_, Xsk_;
  deque_t<Covariance<State>> Pfk_pred_, Pfk_est_, Psk_;
  deque_t<Jacobian<State, State>> Ak_;
};

} // kalmanif

#endif // _KALMANIF_KALMANIF_IMPL_RAUCH_TUNG_STRIEBEL_SMOOTHER_H_
