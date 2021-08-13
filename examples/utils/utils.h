#ifndef _KALMANIF_EXAMPLES_UTILS_H_
#define _KALMANIF_EXAMPLES_UTILS_H_

#include <iostream>
#include <iomanip>

namespace kalmanif {
static void show_usage(const std::string& name) {
  std::cerr << "Usage: " << name << " <option(s)>\n"
            << "E.g. " << name << " -f demo_plots -n\n"
            << "Options:\n"
            << "\t-h, --help\t\tShow this helper\n"
            << "\t-t, --plot-trajectory\tPlot the trajectories\n"
            << "\t-e, --plot-error\tPlot the errors\n"
            << "\t-f, --filename FILENAME\tBase filename to save plots"
            << std::endl;
}
} // namespace kalmanif

#define KALMANIF_DEMO_PROCESS_INPUT(argc, argv)                     \
  bool plot_trajectory = false;                                     \
  bool plot_error = false;                                          \
  bool quiet = false;                                               \
  std::string filename;                                             \
  for (int i = 1; i < argc; ++i) {                                  \
    std::string arg = argv[i];                                      \
    if ((arg == "-h") || (arg == "--help")) {                       \
      kalmanif::show_usage(argv[0]);                                \
      return EXIT_SUCCESS;                                          \
    } else if ((arg == "-t") || (arg == "--plot-trajectory")) {     \
      plot_trajectory = true;                                       \
    } else if ((arg == "-e") || (arg == "--plot-error")) {          \
      plot_trajectory = true;                                       \
    } else if ((arg == "-q") || (arg == "--quiet")) {               \
      quiet = true;                                                 \
    } else if ((arg == "-f") || (arg == "--filename")) {            \
      if (i + 1 < argc) {                                           \
        filename = argv[++i];                                       \
      } else {                                                      \
        std::cerr << "--filename option requires one argument.\n";  \
        return EXIT_FAILURE;                                        \
      }                                                             \
    } else {                                                        \
      std::cerr << "Unknow option '" << argv[i] << "'.\n";          \
    }                                                               \
  }

#define KALMANIF_DEMO_PRETTY_PRINT() \
  std::cout << std::fixed << std::setprecision(5) << std::showpos << std::endl;

#define KALMANIF_DEMO_PRINT_TRAJECTORY(collector)                                       \
  {                                                                                     \
    const auto& Xs_sim = collector.simu().Xs;                                           \
    for (std::size_t i=0; i<Xs_sim.size(); ++i) {                                       \
      std::cout << "X simulated\t: " << Xs_sim[i].log() << "\n";                        \
      for (const auto& c : collector) {                                                 \
        std::cout << "X " << c.first << "\t\t: " << c.second.Xs[i].log()                \
                  << " : |d|=" << (Xs_sim[i] - c.second.Xs[i]).weightedNorm() << "\n";  \
      }                                                                                 \
      std::cout << "-------------------------------------------" << "\n";               \
    }                                                                                   \
  }

#endif // _KALMANIF_EXAMPLES_UTILS_H_