#ifndef _KALMANIF_EXAMPLES_UTILS_PLOTS_H_
#define _KALMANIF_EXAMPLES_UTILS_PLOTS_H_

#ifdef WITH_PLOTS
  #include <sciplot/sciplot.hpp>
#endif

#include <iostream>
#include <map>
#include <numeric>
#include <unordered_map>
#include <Eigen/StdVector>

namespace manif {

template <typename Scalar> struct SE2;
template <typename Scalar> struct SE3;
template <typename Scalar> struct SE_2_3;

} // namespace manif


namespace kalmanif {

template <typename T> using vector_t = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename LieGroup> struct Labeler{
  static std::vector<std::string> labels() { return {}; }
};

template <typename Scalar> struct Labeler<manif::SE2<Scalar>> {
  static std::vector<std::string> labels() {
      return {"x", "y", "theta"};
    }
};

template <typename Scalar> struct Labeler<manif::SE3<Scalar>> {
  static std::vector<std::string> labels() {
      return {"x", "y", "z", "r", "p", "y"};
    }
};

template <typename Scalar> struct Labeler<manif::SE_2_3<Scalar>> {
  static std::vector<std::string> labels() {
      return {"x", "y", "z", "xdot", "ydot", "zdot", "r", "p", "y"};
    }
};

template <typename Scalar, typename T> struct RMSE {
  Scalar operator()(
    const Scalar& lhs, const typename T::Tangent& rhs
  ) const {
    return lhs + rhs.squaredWeightedNorm();
  }
};

template <typename Scalar, typename T> struct ATE;

template <typename Scalar>
struct ATE<Scalar, manif::SE2<Scalar>> {
  Scalar operator()(
    const Scalar& lhs, const typename manif::SE2<Scalar>::Tangent& rhs
  ) const {
    return lhs + rhs.coeffs().template head<2>().squaredNorm();
  }
};

template <typename Scalar>
struct ATE<Scalar, manif::SE3<Scalar>> {
  Scalar operator()(
    const Scalar& lhs, const typename manif::SE3<Scalar>::Tangent& rhs
  ) const {
    return lhs + rhs.coeffs().template head<3>().squaredNorm();
  }
};

template <typename Scalar> struct ATE<Scalar, manif::SE_2_3<Scalar>> {
  Scalar operator()(
    const Scalar& lhs, const typename manif::SE_2_3<Scalar>::Tangent& rhs
  ) const {
    return lhs + rhs.coeffs().template head<3>().squaredNorm();
  }
};

template <typename Scalar, typename T> struct AOE;

template <typename Scalar>
struct AOE<Scalar, manif::SE2<Scalar>> {
  Scalar operator()(
    const Scalar& lhs, const typename manif::SE2<Scalar>::Tangent& rhs
  ) const {
    using std::abs;
    return lhs + abs(rhs.angle());
  }
};

template <typename Scalar>
struct AOE<Scalar, manif::SE3<Scalar>> {
  Scalar operator()(
    const Scalar& lhs, const typename manif::SE3<Scalar>::Tangent& rhs
  ) const {
    static const auto QI = Eigen::Quaternion<Scalar>::Identity();
    return lhs + rhs.exp().quat().angularDistance(QI);
  }
};

template <typename Scalar> struct AOE<Scalar, manif::SE_2_3<Scalar>> {
  Scalar operator()(
    const Scalar& lhs, const typename manif::SE_2_3<Scalar>::Tangent& rhs
  ) const {
    static const auto QI = Eigen::Quaternion<Scalar>::Identity();
    return lhs + rhs.exp().quat().angularDistance(QI);
  }
};

template <typename LieGroup>
struct DataCollector {

  using Scalar = typename LieGroup::Scalar;

  void collect(const LieGroup& X, const Covariance<LieGroup>& P, double t) {
    time.push_back(t);
    Xs.push_back(X);
    Ps.push_back(P);
  }

  void reserve(std::size_t s) {
    time.reserve(s);
    Xs.reserve(s);
    Ps.reserve(s);
  }

  std::string name;
  std::vector<Scalar> time;
  vector_t<LieGroup> Xs;
  vector_t<Covariance<LieGroup>> Ps;
};

template <typename LieGroup>
struct DemoDataCollector {
protected:

  using Collector = DataCollector<LieGroup>;
  using Collectors = std::unordered_map<std::string, Collector>;

public:

  void collect(
    const std::string& name,
    const LieGroup& X,
    const Covariance<LieGroup>& P,
    double t
  ) {
    collectors_[name].collect(X, P, t);
    // @todo fix
    collectors_[name].name = name;
  }

  void collect(const LieGroup& X, double t) {
    collector_true_.collect(X, Covariance<LieGroup>::Zero(), t);
  }

  auto at(const std::string& name) const {
    return collectors_.at(name);
  }

  auto simu() const {
    return collector_true_;
  }

  auto begin() const {
    return collectors_.begin();
  }

  auto end() const {
    return collectors_.end();
  }

  template <typename... Strings>
  void reserve(const std::size_t s, Strings&&... strings) {
    collector_true_.reserve(s);
    (collectors_[std::forward<Strings>(strings)].reserve(s), ...);
  }

protected:

  Collector collector_true_;
  Collectors collectors_;
};

template <typename LieGroup>
struct DemoDataProcessor {
protected:

  using Scalar = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  using Collector = DataCollector<LieGroup>;
  using Collectors = DemoDataCollector<LieGroup>;

public:

  DemoDataProcessor& process(const Collectors& collectors) {
    const auto& collector_true = collectors.simu();
    for (const auto& collector : collectors) {
      KALMANIF_CHECK(collector_true.Xs.size() == collector.second.Xs.size());
      process(collector_true, collector.second);
    }
    return *this;
  }

  void print() const {
    std::cout << "\tRMSE\t\tRRMSE\t\tATE\t\tRTE\t\tAOE\t\tROE\t\tFPE\n";

    for (const auto& f : metrics) {
      std::cout
        << f.first << "\t"
        << f.second.at("RMSE")   << "\t"
        << f.second.at("RRMSE")  << "\t"
        << f.second.at("ATE")    << "\t"
        << f.second.at("RTE")    << "\t"
        << f.second.at("AOE")    << "\t"
        << f.second.at("ROE")    << "\t"
        << f.second.at("FPE")    << "\t"
        << "\n";
    }
    std::cout << "\n----------------------------------\n";
  }

protected:

  std::map<std::string, std::unordered_map<std::string, Scalar>> metrics;

  void process(const Collector& collector_true, const Collector& collector_est) {

    const auto& filter = collector_est.name;

    {
      const auto dXs = computedXs(collector_true.Xs, collector_est.Xs);

      metrics[filter]["RMSE"] = computeRMSE(dXs);
      metrics[filter]["ATE"]  = computeATE(dXs);
      metrics[filter]["AOE"]  = computeAOE(dXs);
    }

    {
      const auto dXs_rel = computeReldXs(
        collector_true.Xs, collector_est.Xs, collector_est.time
      );

      metrics[filter]["RRMSE"] = computeRMSE(dXs_rel);
      metrics[filter]["RTE"]   = computeATE(dXs_rel);
      metrics[filter]["ROE"]   = computeAOE(dXs_rel);
    }

    metrics[filter]["FPE"] = computeFPE(collector_true.Xs, collector_est.Xs);
  }

  vector_t<Tangent> computedXs(
    const vector_t<LieGroup>& Xs_true, const vector_t<LieGroup>& Xs_est
  ) const {
    KALMANIF_CHECK(Xs_true.size() == Xs_est.size());
    vector_t<Tangent> dXs;
    for (std::size_t i=0; i<Xs_true.size(); ++i) {
        const auto& X_true = Xs_true[i];
        const auto& X_est = Xs_est[i];
        dXs.push_back(X_est - X_true);
    }

    return dXs;
  }

  Scalar computeRMSE(const vector_t<Tangent>& dXs) const {
    return std::sqrt(
      std::accumulate(
        dXs.begin(), dXs.end(), Scalar(0), RMSE<Scalar, LieGroup>()
      ) / Scalar(dXs.size())
    );
  }

  Scalar computeATE(const vector_t<Tangent>& dXs) const {
    return std::sqrt(
      std::accumulate(
        dXs.begin(), dXs.end(), Scalar(0), ATE<Scalar, LieGroup>()
      ) / Scalar(dXs.size())
    );
  }

  Scalar computeAOE(const vector_t<Tangent>& dXs) const {
    return std::accumulate(
      dXs.begin(), dXs.end(), Scalar(0), AOE<Scalar, LieGroup>()
    ) / Scalar(dXs.size());
  }

  vector_t<Tangent> computeReldXs(
    const vector_t<LieGroup>& Xs_true,
    const vector_t<LieGroup>& Xs_est,
    const std::vector<double>& time,
    Scalar dt = 1
  ) const {

    KALMANIF_CHECK(Xs_true.size() == Xs_est.size());
    KALMANIF_CHECK(Xs_true.size() == time.size());
    KALMANIF_CHECK(dt >= 0);

    LieGroup X_true_o = Xs_true.front();
    LieGroup X_est_o = Xs_est.front();
    Scalar time_old = time.front();

    vector_t<Tangent> rel;

    for (std::size_t i=1; i<Xs_true.size(); ++i) {

      if ((time[i] - time_old) > dt) {
        X_true_o = Xs_true[i];
        X_est_o = Xs_est[i];
        time_old = time[i];
        continue;
      }

      rel.push_back(X_est_o.between(Xs_est[i]) - X_true_o.between(Xs_true[i]));
    }

    return rel;
  }

  Scalar computeFPE(
    const vector_t<LieGroup>& Xs_true, const vector_t<LieGroup>& Xs_est
  ) const {
    return (Xs_est.back() - Xs_true.back()).weightedNorm();
  }
};

template <typename LieGroup>
struct DataPlotter {
  static void plot(
    const DataCollector<LieGroup>& data_true,
    const DataCollector<LieGroup>& data_est,
    const std::string& filename = "",
    bool show = true
  ) {
#ifdef WITH_PLOTS
    if (filename.empty() && !show) {
      return;
    }

    // See plots parameters at https://stackoverflow.com/a/19420678

    using Scalar = typename LieGroup::Scalar;
    using Tangent = typename LieGroup::Tangent;

    KALMANIF_CHECK(data_true.Xs.size() == data_est.Xs.size());

    // Prepare the data

    vector_t<Tangent> dXs;
    dXs.reserve(data_true.Xs.size());
    std::vector<Scalar> error;
    error.reserve(data_true.Xs.size());
    std::vector<std::vector<Scalar>> cs(LieGroup::DoF);
    std::vector<std::vector<Scalar>> sigma3(LieGroup::DoF);
    for (std::size_t i=0; i<data_true.Xs.size(); ++i) {
      const auto& X_true = data_true.Xs[i];
      const auto& X_est = data_est.Xs[i];
      const auto& P_est = data_est.Ps[i];

      dXs.push_back(X_est - X_true);
      error.push_back(dXs.back().weightedNorm());

      for (int c = 0; c < LieGroup::DoF; ++c) {
        cs[c].push_back(dXs.back().coeffs()(c));
        sigma3[c].push_back(std::sqrt(P_est(c, c)) * Scalar(3));
      }
    }

    const auto labels = Labeler<LieGroup>::labels();

    // Generate plots

    std::vector<sciplot::Plot> plots(LieGroup::DoF + 1);
    for (int i = 0; i < LieGroup::DoF; ++i) {
      plots[i].xlabel("time");
      plots[i].ylabel("error " + labels[i]);
      plots[i].drawCurve(data_est.time, cs[i]).lineColor("blue").lineWidth(1);
      plots[i].drawCurve(
        data_est.time, sigma3[i]
      ).lineColor("red").dashType(12).lineWidth(1);
      std::for_each(sigma3[i].begin(), sigma3[i].end(), [](double &n){ n*=-1.; });
      plots[i].drawCurve(
        data_est.time, sigma3[i]
      ).lineColor("red").dashType(12).lineWidth(1);
      plots[i].legend().hide();
    }

    sciplot::Plot& plot_xi_norm = plots.back();
    plot_xi_norm.xlabel("time");
    plot_xi_norm.ylabel("full-state error");
    plot_xi_norm.drawCurve(data_est.time, error).lineColor("blue").lineWidth(1);
    plot_xi_norm.legend().hide();

    sciplot::Figure fig = {{ plots[0], plots[1] },
                           { plots[2], plots[3] }};

    fig.title(data_est.name + " state error with 3-sigmas");

    if (show) {
      fig.show();
    }

    if (!filename.empty()) {
      fig.save(filename);
    }
#else
  KALMANIF_UNUSED_VARIABLE(data_true);
  KALMANIF_UNUSED_VARIABLE(data_est);
  KALMANIF_UNUSED_VARIABLE(filename);
  KALMANIF_UNUSED_VARIABLE(show);
  std::cerr << "Compiled without plots!\n";
#endif // WITH_PLOTS
  }
};

template <typename LieGroup>
struct DemoDataPlotter {
  static void plot(
    const DemoDataCollector<LieGroup>& collectors,
    const std::string& base_filename = "",
    bool show = true
  ) {
    for (const auto& p : collectors) {
      std::string filename;

      if (!base_filename.empty()) {
        filename = base_filename + "_" + p.first + ".pdf";
      }

      DataPlotter<LieGroup>::plot(collectors.simu(), p.second, filename, show);
    }
  }
};

struct DemoPlotColor {
  static const std::vector<std::string> colors;
};

const std::vector<std::string> DemoPlotColor::colors = {
  "red",
  "blue",
  "purple",
  "cyan",
  "green",
  "yellow",
  "orange",
  "brown",
  "pink"
};

template <typename LieGroup>
struct DemoTrajPlotter {
  static void plot(
    const DemoDataCollector<LieGroup>& collectors,
    const std::string& filename = "",
    bool show = true
  ) {
#ifdef WITH_PLOTS

  constexpr auto Dim = LieGroup::Dim;

  if (filename.empty() && !show) {
    return;
  }

  typename std::conditional<Dim==3, sciplot::Plot3D, sciplot::Plot>::type plot;
  plot.xlabel("x");
  plot.ylabel("y");

  if constexpr (Dim==3) {
    plot.zlabel("z");
  }

  // Parse first collector to retrieve
  // and plot the true trajectory

  std::size_t size = collectors.simu().Xs.size();
  std::size_t step = std::max(size/1000, std::size_t(1)); // show max 1000 poses
  std::vector<double> x_true, y_true, z_true;
  for (std::size_t i=0; i<collectors.simu().Xs.size(); i+=step) {
  // for (const auto& X_true : collectors.simu().Xs) {
    const auto& X_true = collectors.simu().Xs[i];
    x_true.push_back(X_true.x());
    y_true.push_back(X_true.y());
    if constexpr (Dim==3) {
      z_true.push_back(X_true.z());
    }

    if (i+step>size) { // make sure to plot the last pose
      i = size-1;
    }
  }

  if constexpr (Dim==3) {
    plot.drawCurve(x_true, y_true, z_true)
        .lineColor("black").lineWidth(1).label("Groundtruth");
  } else {
    plot.drawCurve(x_true, y_true)
        .lineColor("black").lineWidth(1).label("Groundtruth");
  }

  int c = 0;
  for (const auto& p : collectors) {
    size = p.second.Xs.size();
    step = std::max(size/1000, std::size_t(1)); // show max 1000 poses
    std::vector<double> x_est, y_est, z_est;
    for (std::size_t i=0; i<p.second.Xs.size(); i+=step) {
    // for (const auto& X_est : p.second.Xs) {
      const auto& X_est = p.second.Xs[i];
      x_est.push_back(X_est.x());
      y_est.push_back(X_est.y());
      if constexpr (Dim==3) {
        z_est.push_back(X_est.z());
      }

      if (i+step>size) { // make sure to plot the last pose
      i = size-1;
    }
    }

    if constexpr (Dim==3) {
      plot.drawCurve(x_est, y_est, z_est)
          .lineColor(DemoPlotColor::colors[c++]).dashType(12)
          .lineWidth(1).label(p.second.name);
    } else {
      plot.drawCurve(x_est, y_est)
        .lineColor(DemoPlotColor::colors[c++]).dashType(12)
        .lineWidth(1).label(p.second.name);
    }
  }

  // Set the legend to be on the bottom along the horizontal
  plot.legend()
      .atOutsideBottom()
      .displayHorizontal()
      .displayExpandWidthBy(2);

  if (show) {
    plot.show();
  }

  if (!filename.empty()) {
    plot.save(filename + ".pdf");
  }
#else
  KALMANIF_UNUSED_VARIABLE(collectors);
  KALMANIF_UNUSED_VARIABLE(filename);
  KALMANIF_UNUSED_VARIABLE(show);
  std::cerr << "Compiled without plots!\n";
#endif // WITH_PLOTS
  }
};

} // namespace kalmanif


#endif // _KALMANIF_EXAMPLES_UTILS_PLOTS_H_