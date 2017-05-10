#include "drake/solvers/mathematical_program.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/monomial.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/is_dynamic_castable.h"
#include "drake/common/test/symbolic_test_util.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/test/generic_trivial_costs.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;
using drake::symbolic::Expression;
using drake::symbolic::Formula;
using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;

using std::all_of;
using std::cref;
using std::enable_if;
using std::endl;
using std::is_permutation;
using std::is_same;
using std::make_shared;
using std::map;
using std::move;
using std::numeric_limits;
using std::ostringstream;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::static_pointer_cast;
using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace solvers {
namespace test {

struct Movable {
  Movable() = default;
  Movable(Movable&&) = default;
  Movable(Movable const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Copyable {
  Copyable() = default;
  Copyable(Copyable&&) = delete;
  Copyable(Copyable const&) = default;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Unique {
  Unique() = default;
  Unique(Unique&&) = delete;
  Unique(Unique const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};
// TODO(naveenoid) : tests need to be purged of Random initializations.

// Check the index, type and name etc of the newly added variables.
// This function only works if the only variables contained in @p prog are @p
// var.
template <typename Derived>
void CheckAddedVariable(const MathematicalProgram& prog,
                        const Eigen::MatrixBase<Derived>& var,
                        const string& var_name, bool is_symmetric,
                        MathematicalProgram::VarType type_expected) {
  // Checks the name of the newly added variables.
  ostringstream msg_buff;
  msg_buff << var << endl;
  EXPECT_EQ(msg_buff.str(), var_name);
  // Checks num_vars() function.
  const int num_new_vars =
      is_symmetric ? var.rows() * (var.rows() + 1) / 2 : var.size();
  EXPECT_EQ(prog.num_vars(), num_new_vars);
  // Checks if the newly added variable is symmetric.
  EXPECT_EQ(math::IsSymmetric(var), is_symmetric);
  // Checks the indices of the newly added variables.
  if (is_symmetric) {
    int var_count = 0;
    for (int j = 0; j < var.cols(); ++j) {
      for (int i = j; i < var.rows(); ++i) {
        EXPECT_EQ(prog.FindDecisionVariableIndex(var(i, j)), var_count);
        ++var_count;
      }
    }
  } else {
    for (int i = 0; i < var.rows(); ++i) {
      for (int j = 0; j < var.cols(); ++j) {
        EXPECT_EQ(prog.FindDecisionVariableIndex(var(i, j)),
                  j * var.rows() + i);
      }
    }
  }

  // Checks the type of the newly added variables.
  const auto& variable_types = prog.DecisionVariableTypes();
  for (int i = 0; i < var.rows(); ++i) {
    for (int j = 0; j < var.cols(); ++j) {
      EXPECT_EQ(variable_types[prog.FindDecisionVariableIndex(var(i, j))],
                type_expected);
      EXPECT_EQ(prog.DecisionVariableType(var(i, j)), type_expected);
    }
  }
}

GTEST_TEST(testAddVariable, testAddContinuousVariables1) {
  // Adds a dynamic-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables(2, 3, "X");
  static_assert(is_same<decltype(X), MatrixXDecisionVariable>::value,
                "should be a dynamic sized matrix");
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedVariable(prog, X, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n",
                     false, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable2) {
  // Adds a static-sized matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewContinuousVariables<2, 3>("X");
  static_assert(is_same<decltype(X), MatrixDecisionVariable<2, 3>>::value,
                "should be a static sized matrix");
  CheckAddedVariable(prog, X, "X(0,0) X(0,1) X(0,2)\nX(1,0) X(1,1) X(1,2)\n",
                     false, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable3) {
  // Adds a dynamic-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(4, "x");
  static_assert(is_same<decltype(x), VectorXDecisionVariable>::value,
                "Should be a VectorXDecisionVariable object.");
  EXPECT_EQ(x.rows(), 4);
  CheckAddedVariable(prog, x, "x(0)\nx(1)\nx(2)\nx(3)\n", false,
                     MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddContinuousVariable4) {
  // Adds a static-sized vector of continuous variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  static_assert(is_same<decltype(x), VectorDecisionVariable<4>>::value,
                "Should be a VectorXDecisionVariable object.");
  CheckAddedVariable(prog, x, "x(0)\nx(1)\nx(2)\nx(3)\n", false,
                     MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddSymmetricVariable1) {
  // Adds a static-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables<3>("X");
  static_assert(is_same<decltype(X), MatrixDecisionVariable<3, 3>>::value,
                "should be a MatrixDecisionVariable<3> object");
  CheckAddedVariable(
      prog, X,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)\n",
      true, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddSymmetricVariable2) {
  // Adds a dynamic-sized symmetric matrix of continuous variables.
  MathematicalProgram prog;
  auto X = prog.NewSymmetricContinuousVariables(3, "X");
  static_assert(is_same<decltype(X), MatrixXDecisionVariable>::value,
                "should be a MatrixXDecisionVariable object");
  EXPECT_EQ(X.rows(), 3);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedVariable(
      prog, X,
      "X(0,0) X(1,0) X(2,0)\nX(1,0) X(1,1) X(2,1)\nX(2,0) X(2,1) X(2,2)\n",
      true, MathematicalProgram::VarType::CONTINUOUS);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable1) {
  // Adds a dynamic-sized matrix of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(2, 3, "B");
  static_assert(is_same<decltype(X), MatrixXDecisionVariable>::value,
                "wrong type");
  EXPECT_EQ(X.rows(), 2);
  EXPECT_EQ(X.cols(), 3);
  CheckAddedVariable(prog, X, "B(0,0) B(0,1) B(0,2)\nB(1,0) B(1,1) B(1,2)\n",
                     false, MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable3) {
  // Adds dynamic-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables(4, "B");
  static_assert(is_same<decltype(X), VectorXDecisionVariable>::value,
                "wrong type");
  EXPECT_EQ(X.rows(), 4);
  CheckAddedVariable(prog, X, "B(0)\nB(1)\nB(2)\nB(3)\n", false,
                     MathematicalProgram::VarType::BINARY);
}

GTEST_TEST(testAddVariable, testAddBinaryVariable4) {
  // Adds static-sized vector of binary variables.
  MathematicalProgram prog;
  auto X = prog.NewBinaryVariables<4>("B");
  static_assert(is_same<decltype(X), VectorDecisionVariable<4>>::value,
                "wrong type");
  CheckAddedVariable(prog, X, "B(0)\nB(1)\nB(2)\nB(3)\n", false,
                     MathematicalProgram::VarType::BINARY);
}

template <typename Derived1, typename Derived2>
typename enable_if<is_same<typename Derived1::Scalar, Variable>::value &&
                   is_same<typename Derived2::Scalar, double>::value>::type
CheckGetSolution(const MathematicalProgram& prog,
                 const Eigen::MatrixBase<Derived1>& vars,
                 const Eigen::MatrixBase<Derived2>& val_expected) {
  auto val = prog.GetSolution(vars);
  static_assert(
      is_same<decltype(val), Eigen::Matrix<double, Derived1::RowsAtCompileTime,
                                           Derived1::ColsAtCompileTime>>::value,
      "GetSolution does not return the right type of matrix");
  EXPECT_TRUE(CompareMatrices(val, val_expected));

  // Checks getting solution for a single variable.
  for (int i = 0; i < vars.rows(); ++i) {
    for (int j = 0; j < vars.cols(); ++j) {
      EXPECT_NEAR(prog.GetSolution(vars(i, j)), val(i, j), 1E-14);
    }
  }
}

GTEST_TEST(testGetSolution, testSetSolution1) {
  // Tests setting and getting solution for
  // 1. A static-sized  matrix of decision variables.
  // 2. A dynamic-sized matrix of decision variables.
  // 3. A static-sized  vector of decision variables.
  // 4. A dynamic-sized vector of decision variables.
  MathematicalProgram prog;
  auto X1 = prog.NewContinuousVariables<2, 3>("X");
  auto X2 = prog.NewContinuousVariables(2, 3, "X");
  auto x3 = prog.NewContinuousVariables<4>("x");
  auto x4 = prog.NewContinuousVariables(4, "x");

  Eigen::Matrix<double, 2, 3> X1_value{};
  X1_value << 0, 1, 2, 3, 4, 5;
  Eigen::Matrix<double, 2, 3> X2_value{};
  X2_value = -X1_value;
  Eigen::Vector4d x3_value(3, 4, 5, 6);
  Eigen::Vector4d x4_value = -x3_value;
  for (int i = 0; i < 3; ++i) {
    prog.SetDecisionVariableValues(X1.col(i), X1_value.col(i));
    prog.SetDecisionVariableValues(X2.col(i), X2_value.col(i));
  }
  prog.SetDecisionVariableValues(x3, x3_value);
  prog.SetDecisionVariableValues(x4, x4_value);

  CheckGetSolution(prog, X1, X1_value);
  CheckGetSolution(prog, X2, X2_value);
  CheckGetSolution(prog, x3, x3_value);
  CheckGetSolution(prog, x4, x4_value);

  // Check a variable that is not a decision variable of the mathematical
  // program.
  Variable z1("z1");
  Variable z2("z2");
  EXPECT_THROW(prog.GetSolution(z1), runtime_error);
  EXPECT_THROW(prog.GetSolution(VectorDecisionVariable<2>(z1, z2)),
               runtime_error);
  EXPECT_THROW(prog.GetSolution(VectorDecisionVariable<2>(z1, X1(0, 0))),
               runtime_error);
}

GTEST_TEST(testMathematicalProgram, testAddFunction) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();

  Movable movable;
  prog.AddCost(std::move(movable), x);
  prog.AddCost(Movable(), x);

  Copyable copyable;
  prog.AddCost(copyable, x);

  Unique unique;
  prog.AddCost(cref(unique), x);
  prog.AddCost(make_shared<Unique>(), x);
  prog.AddCost(unique_ptr<Unique>(new Unique), x);
}

GTEST_TEST(testMathematicalProgram, BoundingBoxTest2) {
  // Test the scalar version of the bounding box constraint methods.

  MathematicalProgram prog;
  auto x1 = prog.NewContinuousVariables<2, 2>("x1");
  MatrixXDecisionVariable x2(2, 2);
  x2 = x1;
  VectorDecisionVariable<4> x3;
  x3 << x1.col(0), x1.col(1);
  VectorXDecisionVariable x4(4);
  x4 = x3;
  // Six different ways to construct an equivalent constraint.
  // 1. Imposes constraint on a static-sized matrix of decision variables.
  // 2. Imposes constraint on a list of vectors of decision variables.
  // 3. Imposes constraint on a dynamic-sized matrix of decision variables.
  // 4. Imposes constraint on a static-sized vector of decision variables.
  // 5. Imposes constraint on a dynamic-sized vector of decision variables.
  // 6. Imposes constraint using a vector of lower/upper bound, as compared
  //    to the previous three cases which use a scalar lower/upper bound.
  auto constraint1 = prog.AddBoundingBoxConstraint(0, 1, x1).constraint();
  auto constraint2 =
      prog.AddBoundingBoxConstraint(0, 1, {x1.col(0), x1.col(1)}).constraint();
  auto constraint3 = prog.AddBoundingBoxConstraint(0, 1, x2).constraint();
  auto constraint4 = prog.AddBoundingBoxConstraint(0, 1, x3).constraint();
  auto constraint5 = prog.AddBoundingBoxConstraint(0, 1, x4).constraint();
  auto constraint6 = prog.AddBoundingBoxConstraint(Eigen::Vector4d::Zero(),
                                                   Eigen::Vector4d::Ones(), x3)
                         .constraint();

  // Checks that the bound variables are correct.
  for (const auto& binding : prog.bounding_box_constraints()) {
    EXPECT_EQ(binding.GetNumElements(), 4u);
    VectorDecisionVariable<4> x_expected;
    x_expected << x1(0, 0), x1(1, 0), x1(0, 1), x1(1, 1);
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(binding.variables()(i), x_expected(i));
    }
  }
  EXPECT_TRUE(
      CompareMatrices(constraint1->lower_bound(), constraint2->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->lower_bound(), constraint3->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->lower_bound(), constraint4->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint4->lower_bound(), constraint5->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint5->lower_bound(), constraint6->lower_bound()));

  EXPECT_TRUE(
      CompareMatrices(constraint1->upper_bound(), constraint2->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->upper_bound(), constraint3->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->upper_bound(), constraint4->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint4->upper_bound(), constraint5->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint5->upper_bound(), constraint6->upper_bound()));
}

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added as a derived class
// from Constraint.
void VerifyAddedCost1(const MathematicalProgram& prog,
                      const shared_ptr<Cost>& cost,
                      const Eigen::Ref<const Eigen::VectorXd>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y, y_expected;
  prog.generic_costs().back().constraint()->Eval(x_value, y);
  cost->Eval(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added by converting
// a class to ConstraintImpl through MakeCost.
void VerifyAddedCost2(const MathematicalProgram& prog,
                      const GenericTrivialCost2& cost,
                      const shared_ptr<Cost>& returned_cost,
                      const Eigen::Ref<const Eigen::Vector2d>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y(1), y_expected(1), y_returned;
  prog.generic_costs().back().constraint()->Eval(x_value, y);
  cost.eval<double>(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
  returned_cost->Eval(x_value, y_returned);
  EXPECT_TRUE(CompareMatrices(y, y_returned));
}

GTEST_TEST(testMathematicalProgram, AddCostTest) {
  // Test if the costs are added correctly.

  // There are ways to add a generic cost
  // 1. Add Binding<Constraint>
  // 2. Add shared_ptr<Constraint> on a VectorDecisionVariable object.
  // 3. Add shared_ptr<Constraint> on a VariableRefList.
  // 4. Add a ConstraintImpl object on a VectorDecisionVariable object.
  // 5. Add a ConstraintImpl object on a VariableRefList object.
  // 6. Add a unique_ptr of object that can be converted to a ConstraintImpl
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewContinuousVariables<2>("y");
  // No cost yet.
  int num_generic_costs = 0;
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()), num_generic_costs);
  EXPECT_EQ(prog.linear_costs().size(), 0u);

  shared_ptr<Cost> generic_trivial_cost1 = make_shared<GenericTrivialCost1>();

  // Adds Binding<Constraint>
  prog.AddCost(Binding<Cost>(generic_trivial_cost1,
                             VectorDecisionVariable<3>(x(0), x(1), y(1))));
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(1, 3, 5),
                   num_generic_costs);

  // Adds a std::shared_ptr<Constraint> on a VectorDecisionVariable object.
  prog.AddCost(generic_trivial_cost1,
               VectorDecisionVariable<3>(x(0), x(1), y(1)));
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(1, 2, 3),
                   num_generic_costs);

  // Adds a std::shared_ptr<Constraint> on a VariableRefList object.
  prog.AddCost(generic_trivial_cost1, {x, y.tail<1>()});
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(2, 3, 4),
                   num_generic_costs);

  GenericTrivialCost2 generic_trivial_cost2;

  // Add an object that can be converted to a ConstraintImpl object on a
  // VectorDecisionVariable object.
  auto returned_cost3 =
      prog.AddCost(generic_trivial_cost2, VectorDecisionVariable<2>(x(0), y(1)))
          .constraint();
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost3,
                   Eigen::Vector2d(1, 2), num_generic_costs);

  // Add an object that can be converted to a ConstraintImpl object on a
  // VariableRefList object.
  auto returned_cost4 =
      prog.AddCost(generic_trivial_cost2, {x.head<1>(), y.tail<1>()})
          .constraint();
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost4,
                   Eigen::Vector2d(1, 2), num_generic_costs);
}

void CheckAddedSymbolicLinearCostUserFun(const MathematicalProgram& prog,
                                         const Expression& e,
                                         const Binding<Cost>& binding,
                                         int num_linear_costs) {
  EXPECT_EQ(prog.linear_costs().size(), num_linear_costs);
  EXPECT_EQ(prog.linear_costs().back().constraint(), binding.constraint());
  EXPECT_TRUE(CheckStructuralEquality(prog.linear_costs().back().variables(),
                                      binding.variables()));
  EXPECT_EQ(binding.constraint()->num_constraints(), 1);
  auto cnstr = prog.linear_costs().back().constraint();
  auto vars = prog.linear_costs().back().variables();
  const Expression cx{(cnstr->A() * vars)(0)};
  double constant_term{0};
  if (is_addition(e)) {
    constant_term = get_constant_in_addition(e);
  } else if (is_constant(e)) {
    constant_term = get_constant_value(e);
  }
  EXPECT_TRUE((e - cx).EqualTo(constant_term));
}

void CheckAddedSymbolicLinearCost(MathematicalProgram* prog,
                                  const Expression& e) {
  int num_linear_costs = prog->linear_costs().size();
  auto binding1 = prog->AddLinearCost(e);
  CheckAddedSymbolicLinearCostUserFun(*prog, e, binding1, ++num_linear_costs);
  auto binding2 = prog->AddCost(e);
  CheckAddedSymbolicLinearCostUserFun(*prog, e, binding2, ++num_linear_costs);
}

GTEST_TEST(testMathematicalProgram, AddLinearCostSymbolic) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // Add linear cost 2 * x(0) + 3 * x(1)
  CheckAddedSymbolicLinearCost(&prog, 2 * x(0) + 3 * x(1));
  // Add linear cost x(1)
  CheckAddedSymbolicLinearCost(&prog, +x(1));
  // Add linear cost x(0) + 2
  CheckAddedSymbolicLinearCost(&prog, x(0) + 2);
  // Add linear cost 2 * x(0) + 3 * x(1) + 2
  CheckAddedSymbolicLinearCost(&prog, 2 * x(0) + 3 * x(1) + 2);
  // Add linear cost 2 * x(1)
  CheckAddedSymbolicLinearCost(&prog, 2 * x(1));
  // Add linear (constant) cost 3
  CheckAddedSymbolicLinearCost(&prog, 3);
  // Add linear cost -x(0)
  CheckAddedSymbolicLinearCost(&prog, -x(0));
  // Add linear cost -(x(1) + 3 * x(0))
  CheckAddedSymbolicLinearCost(&prog, -(x(1) + 3 * x(0)));
  // Add linear cost x(1)*x(1) + x(0) - x(1)*x(1)
  CheckAddedSymbolicLinearCost(&prog, x(1) * x(1) + x(0) - x(1) * x(1));
}

void CheckAddedQuadraticCost(MathematicalProgram* prog,
                             const Eigen::MatrixXd& Q, const Eigen::VectorXd& b,
                             const VectorXDecisionVariable& x) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto cnstr = prog->AddQuadraticCost(Q, b, x).constraint();

  EXPECT_EQ(++num_quadratic_cost, prog->quadratic_costs().size());
  // Check if the newly added quadratic constraint, and the returned
  // quadratic constraint, both match 0.5 * x' * Q * x + b' * x
  EXPECT_EQ(cnstr, prog->quadratic_costs().back().constraint());
  EXPECT_EQ(cnstr->Q(), Q);
  EXPECT_EQ(cnstr->b(), b);
}

GTEST_TEST(testMathematicalProgram, AddQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d::Zero(), x);

  CheckAddedQuadraticCost(&prog, Matrix3d::Identity(), Vector3d(1, 2, 3), x);
}

void CheckAddedSymbolicQuadraticCostUserFun(const MathematicalProgram& prog,
                                            const Expression& e,
                                            double constant,
                                            const Binding<Cost>& binding,
                                            int num_quadratic_cost) {
  EXPECT_EQ(num_quadratic_cost, prog.quadratic_costs().size());
  EXPECT_EQ(binding.constraint(), prog.quadratic_costs().back().constraint());
  EXPECT_EQ(binding.variables(), prog.quadratic_costs().back().variables());

  auto cnstr = prog.quadratic_costs().back().constraint();
  // Check the added cost is 0.5 * x' * Q * x + b' * x
  const auto& x_bound = binding.variables();
  const Expression e_added =
      0.5 * x_bound.dot(cnstr->Q() * x_bound) + cnstr->b().dot(x_bound);
  EXPECT_PRED2(ExprEqual, e_added.Expand() + constant, e.Expand());
}

void CheckAddedSymbolicQuadraticCost(MathematicalProgram* prog,
                                     const Expression& e, double constant) {
  int num_quadratic_cost = prog->quadratic_costs().size();
  auto binding1 = prog->AddQuadraticCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, constant, binding1,
                                         ++num_quadratic_cost);
  auto binding2 = prog->AddCost(e);
  CheckAddedSymbolicQuadraticCostUserFun(*prog, e, constant, binding2,
                                         ++num_quadratic_cost);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicQuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  // Identity diagonal term.
  Expression e1 = x.transpose() * x;
  CheckAddedSymbolicQuadraticCost(&prog, e1, 0);

  // Identity diagonal term.
  Expression e2 = x.transpose() * x + 1;
  CheckAddedSymbolicQuadraticCost(&prog, e2, 1);

  // Identity diagonal term.
  Expression e3 = x(0) * x(0) + x(1) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e3, 2);

  // Non-identity diagonal term.
  Expression e4 = x(0) * x(0) + 2 * x(1) * x(1) + 3 * x(2) * x(2) + 3;
  CheckAddedSymbolicQuadraticCost(&prog, e4, 3);

  // Cross terms.
  Expression e5 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0) * x(1) + 2;
  CheckAddedSymbolicQuadraticCost(&prog, e5, 2);

  // Linear terms.
  Expression e6 = x(0) * x(0) + 2 * x(1) * x(1) + 4 * x(0);
  CheckAddedSymbolicQuadraticCost(&prog, e6, 0);

  // Cross terms and linear terms.
  Expression e7 = (x(0) + 2 * x(1) + 3) * (x(0) + x(1) + 4) + 3 * x(0) * x(0) +
                  6 * pow(x(1) + 1, 2);
  CheckAddedSymbolicQuadraticCost(&prog, e7, 18);

  // Cubic polynomial case.
  Expression e8 = pow(x(0), 3) + 1;
  EXPECT_THROW(prog.AddQuadraticCost(e8), runtime_error);
}

GTEST_TEST(testMathematicalProgram, TestL2NormCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // |Ax - b|^2 = (x-xd)'Q(x-xd) => Q = A'*A and b = A*xd.
  Eigen::Matrix2d A;
  A << 1, 2, 3, 4;
  Eigen::Matrix2d Q = A.transpose() * A;
  Eigen::Vector2d x_desired;
  x_desired << 5, 6;
  Eigen::Vector2d b = A * x_desired;

  auto obj1 = prog.AddQuadraticErrorCost(Q, x_desired, x).constraint();
  auto obj2 = prog.AddL2NormCost(A, b, x).constraint();

  // Test the objective at a 6 arbitrary values (to guarantee correctness
  // of the six-parameter quadratic form.
  Eigen::Vector2d x0;
  Eigen::VectorXd y1, y2;
  x0 << 7, 8;

  for (int i = 0; i < 6; i++) {
    obj1->Eval(x0, y1);
    obj2->Eval(x0, y2);

    EXPECT_TRUE(CompareMatrices(y1, y2));
    EXPECT_TRUE(CompareMatrices(
        y2, (A * x0 - b).transpose() * (A * x0 - b) - b.transpose() * b));
    // Note: Currently have to subtract out the constant term (b'*b) due to
    // issue #3500.

    x0 += Eigen::Vector2d::Constant(2);
  }
}

void CheckAddedPolynomialCost(MathematicalProgram* prog, const Expression& e) {
  int num_cost = prog->generic_costs().size();
  const auto binding = prog->AddPolynomialCost(e);
  EXPECT_EQ(prog->generic_costs().size(), ++num_cost);
  EXPECT_EQ(binding.constraint(), prog->generic_costs().back().constraint());
  // Now reconstruct the symbolic expression from `binding`.
  const auto polynomial = binding.constraint()->polynomials()(0);
  symbolic::MonomialToCoefficientMap map_expected;
  for (const auto& m : polynomial.GetMonomials()) {
    map<Variable::Id, int> map_var_to_power;
    for (const auto& term : m.terms) {
      map_var_to_power.emplace(term.var, term.power);
    }
    symbolic::Monomial m_symbolic(map_var_to_power);
    map_expected.emplace(m_symbolic, m.coefficient);
  }
  // Now compare the reconstructed symbolic polynomial with `e`.
  const auto map =
      symbolic::DecomposePolynomialIntoMonomial(e, e.GetVariables());
  EXPECT_EQ(map.size(), map_expected.size());
  for (const auto& m : map) {
    const auto m_expected_it = map_expected.find(m.first);
    EXPECT_NE(m_expected_it, map_expected.end());
    EXPECT_EQ(m_expected_it->second, m.second);
  }
}

GTEST_TEST(testMathematicalProgram, testAddPolynomialCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  // Add a cubic cost
  CheckAddedPolynomialCost(&prog, pow(x(0), 3));

  // Add a cubic cost
  CheckAddedPolynomialCost(&prog, pow(x(0), 2) * x(1));

  // Add a 4th order cost
  CheckAddedPolynomialCost(
      &prog, x(0) * x(0) * x(1) * x(1) + pow(x(0), 3) * x(1) + 2 * x(1));
}

GTEST_TEST(testMathematicalProgram, testAddCostThrowError) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();

  // Add a non-polynomial cost.
  EXPECT_THROW(prog.AddCost(sin(x(0))), runtime_error);

  // Add a cost containing variable not included in the mathematical program.
  Variable y("y");
  EXPECT_THROW(prog.AddCost(x(0) + y), runtime_error);
  EXPECT_THROW(prog.AddCost(x(0) * x(0) + y), runtime_error);
}

GTEST_TEST(testMathematicalProgram, testAddGenericCost) {
  using GenericPtr = shared_ptr<Cost>;
  using Matrix1d = Vector1d;

  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<1>();

  GenericPtr linear_cost(new LinearCost(Matrix1d(1)));
  prog.AddCost(linear_cost, x);
  EXPECT_EQ(prog.linear_costs().size(), 1);

  GenericPtr quadratic_cost(new QuadraticCost(Matrix1d(1), Vector1d(1)));
  prog.AddCost(quadratic_cost, x);
  EXPECT_EQ(prog.quadratic_costs().size(), 1);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
