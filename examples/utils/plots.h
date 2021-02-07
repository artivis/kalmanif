#ifndef _KALMANIF_EXAMPLES_UTILS_PLOTS_H_
#define _KALMANIF_EXAMPLES_UTILS_PLOTS_H_

#ifdef WITH_PLOTS
  #include <sciplot/sciplot.hpp>
#endif

#include <iostream>
#include <map>
#include <numeric>
#include <unordered_map>

namespace manif {

template <typename Scalar> struct SE2;
template <typename Scalar> struct SE3;
template <typename Scalar> struct SE_2_3;

} // namespace manif


namespace kalmanif {

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

  void collect(
    const LieGroup& X_true,
    const LieGroup& X_est,
    const Covariance<LieGroup>& P_est,
    double t
  ) {
    time.push_back(t);
    Xs_true.push_back(X_true);
    Xs_est.push_back(X_est);
    Ps_est.push_back(P_est);
  }

  std::string name;
  std::vector<Scalar> time;
  std::vector<LieGroup> Xs_true;
  std::vector<LieGroup> Xs_est;
  std::vector<Covariance<LieGroup>> Ps_est;
};

template <typename LieGroup>
struct DemoDataCollector {
protected:

  using Collector = DataCollector<LieGroup>;
  using Collectors = std::unordered_map<std::string, Collector>;

public:

   void collect(
    const std::string& name,
    const LieGroup& X_true,
    const LieGroup& X_est,
    const Covariance<LieGroup>& P_est,
    double t
  ) {
    collectors[name].collect(X_true, X_est, P_est, t);
    // @todo fix
    collectors[name].name = name;
  }

  auto at(const std::string& name) const {
    return collectors.at(name);
  }

  auto begin () const {
    return collectors.begin();
  }

  auto end () const {
    return collectors.end();
  }

protected:

  Collectors collectors;
};

template <typename LieGroup>
struct DemoDataProcessor {
protected:

  using Scalar = typename LieGroup::Scalar;
  using Tangent = typename LieGroup::Tangent;

  using Collector = DataCollector<LieGroup>;
  using Collectors = DemoDataCollector<LieGroup>;

public:

  DemoDataProcessor& process(const Collector& collector) {

    const auto& filter = collector.name;

    {
      const auto dXs = computedXs(collector.Xs_true, collector.Xs_est);

      metrics[filter]["RMSE"] = computeRMSE(dXs);
      metrics[filter]["ATE"]  = computeATE(dXs);
      metrics[filter]["AOE"]  = computeAOE(dXs);
    }

    {
      const auto dXs_rel = computeReldXs(collector.Xs_true, collector.Xs_est, collector.time);

      metrics[filter]["RRMSE"] = computeRMSE(dXs_rel);
      metrics[filter]["RTE"]   = computeATE(dXs_rel);
      metrics[filter]["ROE"]   = computeAOE(dXs_rel);
    }

    metrics[filter]["FPE"] = computeFPE(collector.Xs_true, collector.Xs_est);

    return *this;
  }

  DemoDataProcessor& process(const Collectors& collectors) {
    for (const auto& collector : collectors) {
      process(collector.second);
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

  std::vector<Tangent> computedXs(
    const std::vector<LieGroup>& Xs_true, const std::vector<LieGroup>& Xs_est
  ) const {
    KALMANIF_CHECK(Xs_true.size() == Xs_est.size());
    std::vector<Tangent> dXs;
    for (int i=0; i<Xs_true.size(); ++i) {
        const auto& X_true = Xs_true[i];
        const auto& X_est = Xs_est[i];
        dXs.push_back(X_est - X_true);
    }

    return dXs;
  }

  Scalar computeRMSE(const std::vector<Tangent>& dXs) const {
    return std::sqrt(
      std::accumulate(
        dXs.begin(), dXs.end(), Scalar(0), RMSE<Scalar, LieGroup>()
      ) / Scalar(dXs.size())
    );
  }

  Scalar computeATE(const std::vector<Tangent>& dXs) const {
    return std::sqrt(
      std::accumulate(
        dXs.begin(), dXs.end(), Scalar(0), ATE<Scalar, LieGroup>()
      ) / Scalar(dXs.size())
    );
  }

  Scalar computeAOE(const std::vector<Tangent>& dXs) const {
    return std::accumulate(
      dXs.begin(), dXs.end(), Scalar(0), AOE<Scalar, LieGroup>()
    ) / Scalar(dXs.size());
  }

  std::vector<Tangent> computeReldXs(
    const std::vector<LieGroup>& Xs_true,
    const std::vector<LieGroup>& Xs_est,
    const std::vector<double>& time,
    Scalar dt = 1
  ) const {

    KALMANIF_CHECK(Xs_true.size() == Xs_est.size());
    KALMANIF_CHECK(Xs_true.size() == time.size());
    KALMANIF_CHECK(dt >= 0);

    LieGroup X_true_o = Xs_true.front();
    LieGroup X_est_o = Xs_est.front();
    Scalar time_old = time.front();

    std::vector<Tangent> rel;

    for (int i=1; i<Xs_true.size(); ++i) {

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
    const std::vector<LieGroup>& Xs_true, const std::vector<LieGroup>& Xs_est
  ) const {
    return (Xs_est.back() - Xs_true.back()).weightedNorm();
  }
};

template <typename LieGroup>
struct DataPlotter {
  static void plot(
    const DataCollector<LieGroup>& data, const std::string& filename = "", bool show = true
  ) {
#ifdef WITH_PLOTS
    if (filename.empty() && !show) {
      return;
    }

    // See plots parameters at https://stackoverflow.com/a/19420678

    using Scalar = typename LieGroup::Scalar;
    using Tangent = typename LieGroup::Tangent;

    KALMANIF_CHECK(data.Xs_true.size() == data.Xs_est.size());

    // Prepare the data

    std::vector<Tangent> dXs;
    dXs.reserve(data.Xs_true.size());
    std::vector<Scalar> error;
    error.reserve(data.Xs_true.size());
    std::vector<std::vector<Scalar>> cs(LieGroup::DoF);
    std::vector<std::vector<Scalar>> sigma3(LieGroup::DoF);
    for (int i=0; i<data.Xs_true.size(); ++i) {
      const auto& X_true = data.Xs_true[i];
      const auto& X_est = data.Xs_est[i];
      const auto& P_est = data.Ps_est[i];

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
      plots[i].drawCurve(data.time, cs[i]).lineColor("blue").lineWidth(1);
      plots[i].drawCurve(data.time, sigma3[i]).lineColor("red").dashType(12).lineWidth(1);
      std::for_each(sigma3[i].begin(), sigma3[i].end(), [](double &n){ n*=-1.; });
      plots[i].drawCurve(data.time, sigma3[i]).lineColor("red").dashType(12).lineWidth(1);
      plots[i].legend().hide();
    }

    sciplot::Plot& plot_xi_norm = plots.back();
    plot_xi_norm.xlabel("time");
    plot_xi_norm.ylabel("full-state error");
    plot_xi_norm.drawCurve(data.time, error).lineColor("blue").lineWidth(1);
    plot_xi_norm.legend().hide();

    sciplot::Figure fig = {{ plots[0], plots[1] },
                           { plots[2], plots[3] }};

    fig.title(data.name + " state error with 3-sigmas");

    if (show) {
      fig.show();
    }

    if (!filename.empty()) {
      fig.save(filename);
    }
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

      DataPlotter<LieGroup>::plot(p.second, filename, show);
    }
  }
};

template <typename LieGroup>
struct DemoTrajPlotter {
  static void plot(
    const DemoDataCollector<LieGroup>& collectors,
    const std::string& filename = "",
    bool show = true
  ) {
#ifdef WITH_PLOTS
    if (filename.empty() && !show) {
      return;
    }

    sciplot::Plot3D plot;
    plot.xlabel("x");
    plot.ylabel("y");
    plot.ylabel("z");

    // Parse first collector to retrieve
    // and plot the true trajectory
    const auto it = collectors.begin();
    if (it != collectors.end()) {
      std::vector<double> x_true, y_true, z_true;
      for (const auto& X_true : it->second.Xs_true) {
        x_true.push_back(X_true.x());
        y_true.push_back(X_true.y());
        z_true.push_back(X_true.z());
      }

      plot.drawCurve(x_true, y_true, z_true)
          .lineColor("black").lineWidth(1).label("Groundtruth");
    }

    std::vector<std::string> colors = {
      "red",
      "blue",
      "purple",
      "cyan",
      "green",
      "yellow",
      "orange",
      "brown",
    };

    int c = 0;
    for (const auto& p : collectors) {
      std::vector<double> x_est, y_est, z_est;
      for (const auto& X_est : p.second.Xs_est) {
        x_est.push_back(X_est.x());
        y_est.push_back(X_est.y());
        z_est.push_back(X_est.z());
      }

      plot.drawCurve(x_est, y_est, z_est)
          .lineColor(colors[c++]).dashType(12)
          .lineWidth(1).label(p.second.name);
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
#endif // WITH_PLOTS
  }
};

template <typename Scalar>
struct DemoTrajPlotter<manif::SE2<Scalar>> {
  static void plot(
    const DemoDataCollector<manif::SE2<Scalar>>& collectors,
    const std::string& filename = "",
    bool show = true
  ) {
#ifdef WITH_PLOTS
    if (filename.empty() && !show) {
      return;
    }

    sciplot::Plot plot;
    plot.xlabel("x");
    plot.ylabel("y");

    // Parse first collector to retrieve
    // and plot the true trajectory
    const auto it = collectors.begin();
    if (it != collectors.end()) {
      std::vector<double> x_true, y_true;
      for (const auto& X_true : it->second.Xs_true) {
        x_true.push_back(X_true.x());
        y_true.push_back(X_true.y());
      }

      plot.drawCurve(x_true, y_true)
          .lineColor("black").lineWidth(1).label("Groundtruth");
    }

    std::vector<std::string> colors = {
      "red",
      "blue",
      "purple",
      "cyan",
      "green",
      "yellow",
      "orange",
      "brown",
    };

    int c = 0;
    for (const auto& p : collectors) {
      std::vector<double> x_est, y_est;
      for (const auto& X_est : p.second.Xs_est) {
        x_est.push_back(X_est.x());
        y_est.push_back(X_est.y());
      }

      plot.drawCurve(x_est, y_est)
          .lineColor(colors[c++]).dashType(12)
          .lineWidth(1).label(p.second.name);
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
#endif // WITH_PLOTS
  }
};

} // namespace kalmanif


#endif // _KALMANIF_EXAMPLES_UTILS_PLOTS_H_