#ifndef _KALMANIF_EXAMPLES_RAND_H_
#define _KALMANIF_EXAMPLES_RAND_H_

#include <random>
#include <chrono>

namespace {

static std::default_random_engine __generator{
  static_cast<unsigned int>(std::chrono::high_resolution_clock::now().time_since_epoch().count())
};

static std::normal_distribution<> __distribution{0, 1};

// Random double in N(0, 1)
auto __randn = [] (auto) {return __distribution(__generator);};

} // namespace

namespace kalmanif {

/**
 * @brief Random Eigen object with entries in weight * N(0, 1).
 *
 * @tparam EigenType The Eigen type to create.
 * @return An EigenType object randomly filled.
 */
template <typename EigenType>
EigenType randn(const EigenType& weights = EigenType::Constant(1)) {
  return weights * EigenType::Zero().unaryExpr(__randn);
}

} // namespace kalmanif


#endif // _KALMANIF_EXAMPLES_RAND_H_