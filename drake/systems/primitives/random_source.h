#pragma once

#include <random>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A source block which generates random numbers at a fixed sampling interval,
/// with a zero-order hold between samples.  For continuous-time systems, this
/// can be interpreted as a band-limited approximation of continuous white noise
/// (with a power-spectral density of the form Ts * sinc^2( omega * Ts ), where
/// Ts is the sampling interval.
///
/// @tparam Distribution A class modeling the c++ RandomNumberDistribution
/// concept.
///   http://en.cppreference.com/w/cpp/concept/RandomNumberDistribution
///
/// Note: This system is only defined for the double scalar type.
///
/// Note: The hard-coding of (default) distribution parameters is imposed
/// intentionally to simplify analysis (by forcing systems taking noise inputs
/// to implement the shifting/scaling, the system itself contains all of the
/// necessary information for stochastic analysis).
///
/// @ingroup primitive_systems
template <typename Distribution>
class RandomSource : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RandomSource)

  using Generator = std::mt19937;
  struct InternalState {
    Generator generator;
    Distribution distribution;
    InternalState(Generator::result_type seed = Generator::default_seed)
        : generator(seed) {}
  };

  /// Constructs the RandomSource system.
  /// @param num_outputs The dimension of the (single) vector output port.
  /// @param sampling_interval_sec The sampling interval in seconds.
  RandomSource(int num_outputs, double sampling_interval_sec) {
    this->DeclarePeriodicUnrestrictedUpdate(sampling_interval_sec, 0.);
    this->DeclareOutputPort(drake::systems::kVectorValued, num_outputs);
    this->DeclareDiscreteState(num_outputs);
    this->DeclareAbstractState(std::make_unique<Value<InternalState>>());
  }

  /// Initializes the random number generator.
  void set_random_seed(int seed) { seed_ = seed; }

 private:
  // Computes a random number and stores it in the discrete state.
  void DoCalcUnrestrictedUpdate(
      const Context<double>&,
      State<double>* state) const override {
    auto* updates = state->get_mutable_discrete_state();
    InternalState& internal_state =
        state->get_mutable_abstract_state()->get_mutable_value(0)
        .GetMutableValue<InternalState>();
    const int N = updates->size();
    for (int i = 0; i < N; i++) {
      double random_value = internal_state.distribution(internal_state.generator);
      (*updates)[i] = random_value;
    }
  }

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override {
      // TODO(eric.cousineau): Why are AbstractValues's constructors explicit,
      // ifthey are simply taking cast-able pointers?
      return std::make_unique<AbstractValues>(
          std::unique_ptr<AbstractValue>(new Value<InternalState>(seed_)));
    }

  // Output is the zero-order hold of the discrete state.
  void DoCalcOutput(
      const drake::systems::Context<double>& context,
      drake::systems::SystemOutput<double>* output) const override {
    output->GetMutableVectorData(0)->SetFromVector(
        context.get_discrete_state(0)->CopyToVector());
  }

  // Note: currently there is undeclared state in the variables below.
  // TODO(russt): Obtain consistent results across multiple platforms (#4361).
  Generator::result_type seed_{Generator::default_seed};
//  mutable Distribution distribution_;
};

namespace internal {
/// Defines a version of the std::uniform_real_distribution that uses the
/// interval [-1,1] with the default parameters.  This is a more natural
/// distribution for random signals.
class mean_zero_uniform_real_distribution
    : public std::uniform_real_distribution<double> {
 public:
  mean_zero_uniform_real_distribution()
      : std::uniform_real_distribution<double>(-1.0, 1.0) {}
};

}  // namespace internal

/// Generates uniformly distributed random numbers in the interval [-1,1].
///
/// @ingroup primitive_systems
typedef RandomSource<internal::mean_zero_uniform_real_distribution>
    UniformRandomSource;

/// Generates uniformly distributed random numbers with mean zero and unit
/// covariance.
///
/// @ingroup primitive_systems
typedef RandomSource<std::normal_distribution<double>> GaussianRandomSource;

}  // namespace systems
}  // namespace drake
