#pragma once

#include "drake/solvers/function.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {

// TODO(eric.cousineau): Remove stopgap, and actually have Constraint and
// Cost be different classes. Consider using some common evaluation base.

/**
 * Stopgap definitino that permits Cost to use functoinality in Constraint.
 * Using an internal implementation permits child costs to inherit directly
 * from cost, thus be convertible to a cost.
 */
class Cost : public Constraint {
 public:
  Cost(const std::shared_ptr<Constraint>& impl)
      : Constraint(impl->num_constraints(), impl->num_vars()),
        impl_(impl)
  { }
 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              Eigen::VectorXd &y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
              AutoDiffVecXd &y) const override;

  std::shared_ptr<Constraint> impl_;
};

/**
 * Stopgap class to provide functionality as constraint, but allow templates to
 * detect a difference from results from CreateConstraint and CreateCost.
 * @tparam C Constraint type to inherit from.
 */
template<typename C>
class CostShim : public Cost {
 public:
  template<typename... Args>
  CostShim(Args&&... args)
      : Cost(std::make_shared<C>(std::forward<Args>(args)...))
  { };
};

class LinearCost : public CostShim<LinearConstraint> {
 public:
  using CostShim::CostShim;
};

class QuadraticCost : public CostShim<QuadraticConstraint> {
 public:
  // Inherit constructor
  using CostShim::CostShim;
};

class PolynomialCost : public CostShim<PolynomialConstraint> {
 public:
  // Inherit constructor
  using CostShim::CostShim;
};

// TODO(eric.cousineau): This may require the core functionality be
// implemented in a constraint, then shimmed to a Cost
/**
 * A cost that may be specified using a callable object
 * @tparam F The function / functor's type
 */
template <typename F>
class FunctionCost : public Cost {
  F const f_;

 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FunctionCost)

  // Construct by copying from an lvalue.
  template <typename... Args>
  FunctionCost(const F& f, Args&&... args)
      : Cost(detail::FunctionTraits<F>::numOutputs(f),
                   detail::FunctionTraits<F>::numInputs(f),
                   std::forward<Args>(args)...),
        f_(f) {}

  // Construct by moving from an rvalue.
  template <typename... Args>
  FunctionCost(F&& f, Args&&... args)
      : Cost(detail::FunctionTraits<F>::numOutputs(f),
                   detail::FunctionTraits<F>::numInputs(f),
                   std::forward<Args>(args)...),
        f_(std::forward<F>(f)) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd& y) const override {
    y.resize(detail::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 detail::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                 detail::FunctionTraits<F>::numOutputs(f_));
    detail::FunctionTraits<F>::eval(f_, x, y);
  }
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd& y) const override {
    y.resize(detail::FunctionTraits<F>::numOutputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) ==
                 detail::FunctionTraits<F>::numInputs(f_));
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) ==
                 detail::FunctionTraits<F>::numOutputs(f_));
    detail::FunctionTraits<F>::eval(f_, x, y);
  }
};

};  // namespace solvers
}  // namespace drake
