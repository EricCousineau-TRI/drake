#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/solvers/mathematical_program.h"

// Symbol: pydrake_doc
constexpr struct /* pydrake_doc */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::solvers
    struct /* solvers */ {
      // Symbol: drake::solvers::MathematicalProgram
      struct /* MathematicalProgram */ {
        // Source: drake/solvers/mathematical_program.h:140
        const char* doc =
R"""(MathematicalProgram stores the decision variables, the constraints and
costs of an optimization problem. The user can solve the problem by
calling solvers::Solve() function, and obtain the results of the
optimization.)""";
        // Symbol: drake::solvers::MathematicalProgram::AddBoundingBoxConstraint
        struct /* AddBoundingBoxConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1542
          const char* doc_3args_lb_ub_vars =
R"""(Adds bounding box constraints referencing potentially a subset of the
decision variables.

Parameter ``lb``:
    The lower bound.

Parameter ``ub``:
    The upper bound.

Parameter ``vars``:
    Will imposes constraint lb(i) <= vars(i) <= ub(i).

Returns:
    The newly constructed BoundingBoxConstraint.)""";
          // Source: drake/solvers/mathematical_program.h:1553
          const char* doc_3args_lb_ub_var =
R"""(Adds bounds for a single variable.

Parameter ``lb``:
    Lower bound.

Parameter ``ub``:
    Upper bound.

Parameter ``var``:
    The decision variable.)""";
          // Source: drake/solvers/mathematical_program.h:1609
          const char* doc_3args_double_double_constEigenMatrixBase =
R"""(Adds the same scalar lower and upper bound to every variable in
``vars``.

Template parameter ``Derived``:
    An Eigen::Matrix with Variable as the scalar type. The matrix has
    unknown number of columns at compile time, or has more than one
    column.

Parameter ``lb``:
    Lower bound.

Parameter ``ub``:
    Upper bound.

Parameter ``vars``:
    The decision variables.)""";
        } AddBoundingBoxConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddConstraint
        struct /* AddConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1019
          const char* doc_1args_binding =
R"""(Adds a generic constraint to the program. This should only be used if
a more specific type of constraint is not available, as it may require
the use of a significantly more expensive solver.)""";
          // Source: drake/solvers/mathematical_program.h:1038
          const char* doc_3args_e_lb_ub =
R"""(Adds one row of constraint lb <= e <= ub where ``e`` is a symbolic
expression.

Raises:
    RuntimeError if 1. ``lb <= e <= ub`` is a trivial constraint such
    as 1 <= 2 <= 3. 2. ``lb <= e <= ub`` is unsatisfiable such as 1 <=
    -5 <= 3

Parameter ``e``:
    A symbolic expression of the the decision variables.

Parameter ``lb``:
    A scalar, the lower bound.

Parameter ``ub``:
    A scalar, the upper bound.

The resulting constraint may be a BoundingBoxConstraint,
LinearConstraint, LinearEqualityConstraint, or ExpressionConstraint,
depending on the arguments. Constraints of the form x == 1 (which
could be created as a BoundingBoxConstraint or
LinearEqualityConstraint) will be constructed as a
LinearEqualityConstraint.)""";
          // Source: drake/solvers/mathematical_program.h:1082
          const char* doc_1args_f =
R"""(Add a constraint represented by a symbolic formula to the program. The
input formula ``f`` can be of the following forms:

1. e1 <= e2
2. e1 >= e2
3. e1 == e2
4. A conjunction of relational formulas where each conjunct is
   a relational formula matched by 1, 2, or 3.

Note that first two cases might return an object of
Binding<BoundingBoxConstraint>, Binding<LinearConstraint>, or
Binding<ExpressionConstraint>, depending on ``f``. Also the third case
might return an object of Binding<LinearEqualityConstraint> or
Binding<ExpressionConstraint>.

It throws an exception if 1. ``f`` is not matched with one of the
above patterns. Especially, strict inequalities (<, >) are not
allowed. 2. ``f`` is either a trivial constraint such as "1 <= 2" or
an unsatisfiable constraint such as "2 <= 1". 3. It is not possible to
find numerical bounds of ``e1`` and ``e2`` where ``f`` = e1 ≃ e2. We
allow ``e1`` and ``e2`` to be infinite but only if there are no other
terms. For example, ``x <= ∞`` is allowed. However, ``x - ∞ <= 0`` is
not allowed because ``x ↦ ∞`` introduces ``nan`` in the evaluation.)""";
        } AddConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddCost
        struct /* AddCost */ {
          // Source: drake/solvers/mathematical_program.h:723
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Adds a generic cost to the optimization program.)""";
        } AddCost;
        // Symbol: drake::solvers::MathematicalProgram::AddDecisionVariables
        struct /* AddDecisionVariables */ {
          // Source: drake/solvers/mathematical_program.h:400
          const char* doc =
R"""(Appends new variables to the end of the existing variables.

Parameter ``decision_variables``:
    The newly added decision_variables.

Precondition:
    ``decision_variables`` should not intersect with the existing
    variables or indeterminates in the optimization program.

Precondition:
    Each entry in ``decision_variables`` should not be a dummy
    variable.

Raises:
    RuntimeError if the preconditions are not satisfied.)""";
        } AddDecisionVariables;
        // Symbol: drake::solvers::MathematicalProgram::AddEqualityConstraintBetweenPolynomials
        struct /* AddEqualityConstraintBetweenPolynomials */ {
          // Source: drake/solvers/mathematical_program.h:2222
          const char* doc =
R"""(Constraining that two polynomials are the same (i.e., they have the
same coefficients for each monomial). This function is often used in
sum-of-squares optimization. We will impose the linear equality
constraint that the coefficient of a monomial in ``p1`` is the same as
the coefficient of the same monomial in ``p2``.

Parameter ``p1``:
    Note that p1's indeterminates should have been registered as
    indeterminates in this MathematicalProgram object, and p1's
    coefficients are affine functions of decision variables in this
    MathematicalProgram object.

Parameter ``p2``:
    Note that p2's indeterminates should have been registered as
    indeterminates in this MathematicalProgram object, and p2's
    coefficients are affine functions of decision variables in this
    MathematicalProgram object.)""";
        } AddEqualityConstraintBetweenPolynomials;
        // Symbol: drake::solvers::MathematicalProgram::AddExponentialConeConstraint
        struct /* AddExponentialConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h:2247
          const char* doc_3args =
R"""(Adds an exponential cone constraint, that z = A * vars + b should be
in the exponential cone. Namely {z₀, z₁, z₂ | z₀ ≥ z₁ * exp(z₂ / z₁),
z₁ > 0}.

Parameter ``A``:
    The A matrix in the documentation above. A must have 3 rows.

Parameter ``b``:
    The b vector in the documentation above.

Parameter ``vars``:
    The variables bound with this constraint.)""";
          // Source: drake/solvers/mathematical_program.h:2257
          const char* doc_1args =
R"""(Add the constraint that z is in the exponential cone.

Parameter ``z``:
    The expression in the exponential cone.

Precondition:
    each entry in ``z`` is a linear expression of the decision
    variables.)""";
        } AddExponentialConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddIndeterminates
        struct /* AddIndeterminates */ {
          // Source: drake/solvers/mathematical_program.h:676
          const char* doc =
R"""(Adds indeterminates. This method appends some indeterminates to the
end of the program's old indeterminates.

Parameter ``new_indeterminates``:
    The indeterminates to be appended to the program's old
    indeterminates.

Precondition:
    ``new_indeterminates`` should not intersect with the program's old
    indeterminates or decision variables.

Precondition:
    Each entry in new_indeterminates should not be dummy.

Precondition:
    Each entry in new_indeterminates should be of CONTINUOUS type.)""";
        } AddIndeterminates;
        // Symbol: drake::solvers::MathematicalProgram::AddL2NormCost
        struct /* AddL2NormCost */ {
          // Source: drake/solvers/mathematical_program.h:881
          const char* doc =
R"""(Adds a cost term of the form | Ax - b |^2.)""";
        } AddL2NormCost;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearComplementarityConstraint
        struct /* AddLinearComplementarityConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1953
          const char* doc =
R"""(Adds a linear complementarity constraints referencing a subset of the
decision variables.)""";
        } AddLinearComplementarityConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearConstraint
        struct /* AddLinearConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1160
          const char* doc_4args_A_lb_ub_vars =
R"""(Adds linear constraints referencing potentially a subset of the
decision variables (defined in the vars parameter).)""";
          // Source: drake/solvers/mathematical_program.h:1188
          const char* doc_4args_a_lb_ub_vars =
R"""(Adds one row of linear constraint referencing potentially a subset of
the decision variables (defined in the vars parameter). lb <= a*vars
<= ub

Parameter ``a``:
    A row vector.

Parameter ``lb``:
    A scalar, the lower bound.

Parameter ``ub``:
    A scalar, the upper bound.

Parameter ``vars``:
    The decision variables on which to impose the linear constraint.)""";
          // Source: drake/solvers/mathematical_program.h:1223
          const char* doc_3args_e_lb_ub =
R"""(Adds one row of linear constraint lb <= e <= ub where ``e`` is a
symbolic expression.

Raises:
    RuntimeError if 1. ``e`` is a non-linear expression. 2. ``lb <= e
    <= ub`` is a trivial constraint such as 1 <= 2 <= 3. 3. ``lb <= e
    <= ub`` is unsatisfiable such as 1 <= -5 <= 3

Parameter ``e``:
    A linear symbolic expression in the form of ``c0 + c1 * v1 + ... +
    cn * vn`` where ``c_i`` is a constant and @v_i is a variable.

Parameter ``lb``:
    A scalar, the lower bound.

Parameter ``ub``:
    A scalar, the upper bound.)""";
          // Source: drake/solvers/mathematical_program.h:1231
          const char* doc_3args_v_lb_ub =
R"""(Adds linear constraints represented by symbolic expressions to the
program. It throws if @v includes a non-linear expression or ``lb <= v
<= ub`` includes trivial/unsatisfiable constraints.)""";
          // Source: drake/solvers/mathematical_program.h:1261
          const char* doc_1args_f =
R"""(Add a linear constraint represented by a symbolic formula to the
program. The input formula ``f`` can be of the following forms:

1. e1 <= e2
2. e1 >= e2
3. e1 == e2
4. A conjunction of relational formulas where each conjunct is
   a relational formula matched by 1, 2, or 3.

Note that first two cases might return an object of
Binding<BoundingBoxConstraint> depending on ``f``. Also the third case
returns an object of Binding<LinearEqualityConstraint>.

It throws an exception if 1. ``f`` is not matched with one of the
above patterns. Especially, strict inequalities (<, >) are not
allowed. 2. ``f`` includes a non-linear expression. 3. ``f`` is either
a trivial constraint such as "1 <= 2" or an unsatisfiable constraint
such as "2 <= 1". 4. It is not possible to find numerical bounds of
``e1`` and ``e2`` where ``f`` = e1 ≃ e2. We allow ``e1`` and ``e2`` to
be infinite but only if there are no other terms. For example, ``x <=
∞`` is allowed. However, ``x - ∞ <= 0`` is not allowed because ``x ↦
∞`` introduces ``nan`` in the evaluation.)""";
          // Source: drake/solvers/mathematical_program.h:1292
          const char* doc_1args_constEigenArrayBase =
R"""(Add a linear constraint represented by an
Eigen::Array<symbolic::Formula> to the program. A common use-case of
this function is to add a linear constraint with the element-wise
comparison between two Eigen matrices, using ``A.array() <=
B.array()``. See the following example.


::

    MathematicalProgram prog;
      Eigen::Matrix<double, 2, 2> A;
      auto x = prog.NewContinuousVariables(2, "x");
      Eigen::Vector2d b;
      ... // set up A and b
      prog.AddLinearConstraint((A * x).array() <= b.array());

A formula in ``formulas`` can be of the following forms:

1. e1 <= e2
 2. e1 >= e2
 3. e1 == e2

It throws an exception if AddLinearConstraint(const symbolic::Formula&
f) throws an exception for f ∈ ``formulas``.

Template parameter ``Derived``:
    An Eigen Array type of Formula.)""";
        } AddLinearConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearCost
        struct /* AddLinearCost */ {
          // Source: drake/solvers/mathematical_program.h:809
          const char* doc_1args =
R"""(Adds a linear cost term of the form a'*x + b.

Parameter ``e``:
    A linear symbolic expression.

Precondition:
    e is a linear expression a'*x + b, where each entry of x is a
    decision variable in the mathematical program.

Returns:
    The newly added linear constraint, together with the bound
    variables.)""";
          // Source: drake/solvers/mathematical_program.h:816
          const char* doc_3args =
R"""(Adds a linear cost term of the form a'*x + b. Applied to a subset of
the variables and pushes onto the linear cost data structure.)""";
          // Source: drake/solvers/mathematical_program.h:836
          const char* doc_2args =
R"""(Adds a linear cost term of the form a'*x. Applied to a subset of the
variables and pushes onto the linear cost data structure.)""";
        } AddLinearCost;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearEqualityConstraint
        struct /* AddLinearEqualityConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1328
          const char* doc_2args =
R"""(Adds one row of linear constraint e = b where ``e`` is a symbolic
expression.

Raises:
    RuntimeError if 1. ``e`` is a non-linear expression. 2. ``e`` is a
    constant.

Parameter ``e``:
    A linear symbolic expression in the form of ``c0 + c1 * x1 + ... +
    cn * xn`` where ``c_i`` is a constant and @x_i is a variable.

Parameter ``b``:
    A scalar.

Returns:
    The newly added linear equality constraint, together with the
    bound variable.)""";
          // Source: drake/solvers/mathematical_program.h:1341
          const char* doc_1args =
R"""(Adds a linear equality constraint represented by a symbolic formula to
the program. The input formula ``f`` is either an equality formula
(``e1 == e2``) or a conjunction of equality formulas.

It throws an exception if

1. ``f`` is neither an equality formula nor a conjunction of equalities.
2. ``f`` includes a non-linear expression.)""";
          // Source: drake/solvers/mathematical_program.h:1452
          const char* doc_3args =
R"""(AddLinearEqualityConstraint

Adds linear equality constraints referencing potentially a subset of
the decision variables.

Example: to add two equality constraints which only depend on two of
the elements of x, you could use


::

    auto x = prog.NewContinuousVariables(6,"myvar");
      Eigen::Matrix2d Aeq;
      Aeq << -1, 2,
              1, 1;
      Eigen::Vector2d beq(1, 3);
      // Imposes constraint
      // -x(0) + 2x(1) = 1
      //  x(0) +  x(1) = 3
      prog.AddLinearEqualityConstraint(Aeq, beq, x.head<2>());)""";
        } AddLinearEqualityConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLinearMatrixInequalityConstraint
        struct /* AddLinearMatrixInequalityConstraint */ {
          // Source: drake/solvers/mathematical_program.h:2067
          const char* doc =
R"""(Adds a linear matrix inequality constraint to the program.)""";
        } AddLinearMatrixInequalityConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddLorentzConeConstraint
        struct /* AddLorentzConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1654
          const char* doc =
R"""(Adds Lorentz cone constraint referencing potentially a subset of the
decision variables.

Parameter ``v``:
    An Eigen::Vector of symbolic::Expression. Constraining that

.. math:: v_0 \ge \sqrt{v_1^2 + ... + v_{n-1}^2}

Returns:
    The newly constructed Lorentz cone constraint with the bounded
    variables.)""";
        } AddLorentzConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddMaximizeGeometricMeanCost
        struct /* AddMaximizeGeometricMeanCost */ {
          // Source: drake/solvers/mathematical_program.h:995
          const char* doc_3args =
R"""(An overloaded version of maximize_geometric_mean.

Precondition:
    A.rows() == b.rows(), A.rows() >= 2.)""";
          // Source: drake/solvers/mathematical_program.h:1009
          const char* doc_2args =
R"""(An overloaded version of maximize_geometric_mean. We add the cost to
maximize the geometric mean of x, i.e., c*power(∏ᵢx(i), 1/n).

Parameter ``c``:
    The positive coefficient of the geometric mean cost, $*Default:*
    is 1.

Precondition:
    x.rows() >= 2.

Precondition:
    c > 0.)""";
        } AddMaximizeGeometricMeanCost;
        // Symbol: drake::solvers::MathematicalProgram::AddMaximizeLogDeterminantSymmetricMatrixCost
        struct /* AddMaximizeLogDeterminantSymmetricMatrixCost */ {
          // Source: drake/solvers/mathematical_program.h:963
          const char* doc =
R"""(Adds the cost to maximize the log determinant of symmetric matrix X.
log(det(X)) is a concave function of X, so we can maximize it through
convex optimization. In order to do that, we introduce slack variables
t, and a lower triangular matrix Z, with the constraints ⌈X Z⌉ is
positive semidifinite. ⌊Zᵀ diag(Z)⌋ log(Z(i, i)) >= t(i) and we will
minimize -∑ᵢt(i).

Parameter ``X``:
    A symmetric positive semidefinite matrix X, whose log(det(X)) will
    be maximized.

Precondition:
    X is a symmetric matrix.

Note:
    The constraint log(Z(i, i)) >= t(i) is imposed as an exponential
    cone constraint. Please make sure your have a solver that supports
    exponential cone constraint (currently SCS does).)""";
        } AddMaximizeLogDeterminantSymmetricMatrixCost;
        // Symbol: drake::solvers::MathematicalProgram::AddPolynomialConstraint
        struct /* AddPolynomialConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1973
          const char* doc =
R"""(Adds a polynomial constraint to the program referencing a subset of
the decision variables (defined in the vars parameter).)""";
        } AddPolynomialConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddPolynomialCost
        struct /* AddPolynomialCost */ {
          // Source: drake/solvers/mathematical_program.h:934
          const char* doc =
R"""(Adds a cost term in the polynomial form.

Parameter ``e``:
    A symbolic expression in the polynomial form.

Returns:
    The newly created cost and the bound variables.)""";
        } AddPolynomialCost;
        // Symbol: drake::solvers::MathematicalProgram::AddPositiveDiagonallyDominantMatrixConstraint
        struct /* AddPositiveDiagonallyDominantMatrixConstraint */ {
          // Source: drake/solvers/mathematical_program.h:2104
          const char* doc =
R"""(Adds the constraint that a symmetric matrix is diagonally dominant
with non-negative diagonal entries. A symmetric matrix X is diagonally
dominant with non-negative diagonal entries if X(i, i) >= ∑ⱼ |X(i, j)|
∀ j ≠ i namely in each row, the diagonal entry is larger than the sum
of the absolute values of all other entries in the same row. A matrix
being diagonally dominant with non-negative diagonals is a sufficient
(but not necessary) condition of a matrix being positive semidefinite.
Internally we will create a matrix Y as slack variables, such that
Y(i, j) represents the absolute value |X(i, j)| ∀ j ≠ i. The diagonal
entries Y(i, i) = X(i, i) The users can refer to "DSOS and SDSOS
Optimization: More Tractable Alternatives to Sum of Squares and
Semidefinite Optimization" by Amir Ali Ahmadi and Anirudha Majumdar,
with arXiv link https://arxiv.org/abs/1706.02586

Parameter ``X``:
    The matrix X. We will use 0.5(X+Xᵀ) as the "symmetric version" of
    X.

Returns:
    Y The slack variable. Y(i, j) represents |X(i, j)| ∀ j ≠ i, with
    the constraint Y(i, j) >= X(i, j) and Y(i, j) >= -X(i, j). Y is a
    symmetric matrix. The diagonal entries Y(i, i) = X(i, i))""";
        } AddPositiveDiagonallyDominantMatrixConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddPositiveSemidefiniteConstraint
        struct /* AddPositiveSemidefiniteConstraint */ {
          // Source: drake/solvers/mathematical_program.h:2016
          const char* doc_1args_symmetric_matrix_var =
R"""(Adds a positive semidefinite constraint on a symmetric matrix.

Raises:
    RuntimeError in Debug mode if ``symmetric_matrix_var`` is not
    symmetric.

Parameter ``symmetric_matrix_var``:
    A symmetric MatrixDecisionVariable object.)""";
          // Source: drake/solvers/mathematical_program.h:2035
          const char* doc_1args_constEigenMatrixBase =
R"""(Adds a positive semidefinite constraint on a symmetric matrix of
symbolic expressions ``e``. We create a new symmetric matrix of
variables M being positive semidefinite, with the linear equality
constraint e == M.

Template parameter ``Derived``:
    An Eigen Matrix of symbolic expressions.

Parameter ``e``:
    Imposes constraint "e is positive semidefinite".

Precondition:
    {1. e is symmetric. 2. e(i, j) is linear for all i, j }

Returns:
    The newly added positive semidefinite constraint, with the bound
    variable M that are also newly added.)""";
        } AddPositiveSemidefiniteConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddQuadraticCost
        struct /* AddQuadraticCost */ {
          // Source: drake/solvers/mathematical_program.h:857
          const char* doc_1args =
R"""(Add a quadratic cost term of the form 0.5*x'*Q*x + b'*x + c. Notice
that in the optimization program, the constant term ``c`` in the cost
is ignored.

Parameter ``e``:
    A quadratic symbolic expression.

Raises:
    std::runtime error if the expression is not quadratic.

Returns:
    The newly added cost together with the bound variables.)""";
          // Source: drake/solvers/mathematical_program.h:924
          const char* doc_3args =
R"""(Adds a cost term of the form 0.5*x'*Q*x + b'x Applied to subset of the
variables.)""";
        } AddQuadraticCost;
        // Symbol: drake::solvers::MathematicalProgram::AddQuadraticErrorCost
        struct /* AddQuadraticErrorCost */ {
          // Source: drake/solvers/mathematical_program.h:862
          const char* doc =
R"""(Adds a cost term of the form (x-x_desired)'*Q*(x-x_desired).)""";
        } AddQuadraticErrorCost;
        // Symbol: drake::solvers::MathematicalProgram::AddRotatedLorentzConeConstraint
        struct /* AddRotatedLorentzConeConstraint */ {
          // Source: drake/solvers/mathematical_program.h:1826
          const char* doc_4args_linear_expression1_linear_expression2_quadratic_expression_tol =
R"""(Adds rotated Lorentz cone constraint on the linear expression v1, v2
and quadratic expression u, such that v1 * v2 >= u, v1 >= 0, v2 >= 0

Parameter ``linear_expression1``:
    The linear expression v1.

Parameter ``linear_expression2``:
    The linear expression v2.

Parameter ``quadratic_expression``:
    The quadratic expression u.

Parameter ``tol``:
    The tolerance to determine if the matrix in v2 is positive
    semidefinite or not.

See also:
    DecomposePositiveQuadraticForm for more explanation. $*Default:*
    is 0.

Returns ``binding``:
    The newly added rotated Lorentz cone constraint, together with the
    bound variables.

Precondition:
1. ``linear_expression1`` is a linear (affine) expression, in the form of
   v1 = c1'*x + d1.
2. ``linear_expression2`` is a linear (affine) expression, in the form of
   v2 = c2'*x + d2.
2. ``quadratic_expression`` is a quadratic expression, in the form of
   


::

    u = x'*Q*x + b'x + a

Also the quadratic expression has to be convex, namely Q is a positive
semidefinite matrix, and the quadratic expression needs to be
non-negative for any x.

Raises:
    RuntimeError if the preconditions are not satisfied.)""";
          // Source: drake/solvers/mathematical_program.h:1844
          const char* doc_1args_v =
R"""(Adds a constraint that a symbolic expression

Parameter ``v``:
    is in the rotated Lorentz cone, i.e.,

.. math:: v_0v_1 \ge v_2^2 + ... + v_{n-1}^2\ v_0 \ge 0, v_1 \ge 0

Parameter ``v``:
    A linear expression of variables, :math:`v = A x + b`, where
    :math:`A, b` are given matrices of the correct size, :math:`x` is
    the vector of decision variables.

Returns ``binding``:
    The newly added rotated Lorentz cone constraint, together with the
    bound variables.)""";
          // Source: drake/solvers/mathematical_program.h:1866
          const char* doc_3args_A_b_vars =
R"""(Adds a rotated Lorentz cone constraint referencing potentially a
subset of decision variables, The linear expression :math:`z=Ax+b` is
in rotated Lorentz cone. A vector :math:`z \in\mathbb{R}^n` is in the
rotated Lorentz cone, if

.. math:: z_0z_1 \ge z_2^2 + ... + z_{n-1}^2

where :math:`A\in\mathbb{R}^{n\times m}, b\in\mathbb{R}^n` are given
matrices.

Parameter ``A``:
    A matrix whose number of columns equals to the size of the
    decision variables.

Parameter ``b``:
    A vector whose number of rows equals to the size fo the decision
    variables.

Parameter ``vars``:
    The decision variables on which the constraint is imposed.)""";
          // Source: drake/solvers/mathematical_program.h:1911
          const char* doc_1args_vars =
R"""(Impose that a vector :math:`x\in\mathbb{R}^m` is in rotated Lorentz
cone. Namely

.. math:: x_0 x_1 \ge x_2^2 + ... + x_{m-1}^2\ x_0 \ge 0, x_1 \ge 0

Parameter ``vars``:
    The stacked column of vars lies in the rotated Lorentz cone.

Returns:
    The newly added rotated Lorentz cone constraint.)""";
        } AddRotatedLorentzConeConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddScaledDiagonallyDominantMatrixConstraint
        struct /* AddScaledDiagonallyDominantMatrixConstraint */ {
          // Source: drake/solvers/mathematical_program.h:2143
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(This is an overloaded variant of addsdd "scaled diagonally dominant
matrix constraint"

Parameter ``X``:
    The matrix X to be constrained scaled diagonally dominant. X.

Precondition:
    X(i, j) should be a linear expression of decision variables.

Returns:
    M A vector of vectors of 2 x 2 symmetric matrices M. For i < j,
    M[i][j] is


::

    [Mⁱʲ(i, i), Mⁱʲ(i, j)]
    [Mⁱʲ(i, j), Mⁱʲ(j, j)].

Note that M[i][j](0, 1) = Mⁱʲ(i, j) = (X(i, j) + X(j, i)) / 2 for i >=
j, M[i][j] is the zero matrix.)""";
        } AddScaledDiagonallyDominantMatrixConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddSosConstraint
        struct /* AddSosConstraint */ {
          // Source: drake/solvers/mathematical_program.h:2166
          const char* doc_2args_p_monomial_basis =
R"""(Adds constraints that a given polynomial ``p`` is a sums-of-squares
(SOS), that is, ``p`` can be decomposed into ``mᵀQm``, where m is the
``monomial_basis``. It returns the coefficients matrix Q, which is
positive semidefinite.)""";
          // Source: drake/solvers/mathematical_program.h:2180
          const char* doc_1args_p =
R"""(Adds constraints that a given polynomial ``p`` is a sums-of-squares
(SOS), that is, ``p`` can be decomposed into ``mᵀQm``, where m is a
monomial basis selected from the sparsity of ``p``. It returns a pair
of constraint bindings expressing:

- The coefficients matrix Q, which is positive semidefinite.
 - The monomial basis m.)""";
          // Source: drake/solvers/mathematical_program.h:2190
          const char* doc_2args_e_monomial_basis =
R"""(Adds constraints that a given symbolic expression ``e`` is a
sums-of-squares (SOS), that is, ``p`` can be decomposed into ``mᵀQm``,
where m is the ``monomial_basis``. Note that it decomposes ``e`` into
a polynomial with respect to ``indeterminates()`` in this mathematical
program. It returns the coefficients matrix Q, which is positive
semidefinite.)""";
          // Source: drake/solvers/mathematical_program.h:2204
          const char* doc_1args_e =
R"""(Adds constraints that a given symbolic expression ``e`` is a
sums-of-squares (SOS), that is, ``e`` can be decomposed into ``mᵀQm``.
Note that it decomposes ``e`` into a polynomial with respect to
``indeterminates()`` in this mathematical program. It returns a pair
expressing:

- The coefficients matrix Q, which is positive semidefinite.
 - The monomial basis m.)""";
        } AddSosConstraint;
        // Symbol: drake::solvers::MathematicalProgram::AddVisualizationCallback
        struct /* AddVisualizationCallback */ {
          // Source: drake/solvers/mathematical_program.h:693
          const char* doc =
R"""(Adds a callback method to visualize intermediate results of the
optimization.

Note:
    Just like other costs/constraints, not all solvers support
    callbacks. Adding a callback here will force
    MathematicalProgram::Solve to select a solver that support
    callbacks. For instance, adding a visualization callback to a
    quadratic programming problem may result in using a nonlinear
    programming solver as the default solver.

Parameter ``callback``:
    a std::function that accepts an Eigen::Vector of doubles
    representing the bound decision variables.

Parameter ``vars``:
    the decision variables that should be passed to the callback.)""";
        } AddVisualizationCallback;
        // Symbol: drake::solvers::MathematicalProgram::Clone
        struct /* Clone */ {
          // Source: drake/solvers/mathematical_program.h:169
          const char* doc =
R"""(Clones an optimization program. The clone will be functionally
equivalent to the source program with the same:

- decision variables
- constraints
- costs
- solver settings
- initial guess

However, the clone's x values will be initialized to NaN, and all
internal solvers will be freshly constructed.

Returns ``new_prog``:
    . The newly constructed mathematical program.)""";
        } Clone;
        // Symbol: drake::solvers::MathematicalProgram::EvalBinding
        struct /* EvalBinding */ {
          // Source: drake/solvers/mathematical_program.h:2585
          const char* doc =
R"""(Evaluates the value of some binding, for some input value for all
decision variables.

Parameter ``binding``:
    A Binding whose variables are decision variables in this program.

Parameter ``prog_var_vals``:
    The value of all the decision variables in this program.

Raises:
    RuntimeError if the size of ``prog_var_vals`` is invalid.)""";
        } EvalBinding;
        // Symbol: drake::solvers::MathematicalProgram::EvalBindingAtInitialGuess
        struct /* EvalBindingAtInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h:2675
          const char* doc =
R"""(Evaluates the evaluator in ``binding`` at the initial guess.

Returns:
    The value of ``binding`` at the initial guess.)""";
        } EvalBindingAtInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::EvalBindings
        struct /* EvalBindings */ {
          // Source: drake/solvers/mathematical_program.h:2617
          const char* doc =
R"""(Evaluates a set of bindings (plural version of ``EvalBinding``).

Parameter ``bindings``:
    List of bindings.

Parameter ``prog``:
    $Parameter ``prog_var_vals``:

The value of all the decision variables in this program.

Returns:
    All binding values, concatenated into a single vector.

Raises:
    RuntimeError if the size of ``prog_var_vals`` is invalid.)""";
        } EvalBindings;
        // Symbol: drake::solvers::MathematicalProgram::EvalVisualizationCallbacks
        struct /* EvalVisualizationCallbacks */ {
          // Source: drake/solvers/mathematical_program.h:2644
          const char* doc =
R"""(Evaluates all visualization callbacks registered with the
MathematicalProgram.

Parameter ``prog_var_vals``:
    The value of all the decision variables in this program.

Raises:
    RuntimeError if the size does not match.)""";
        } EvalVisualizationCallbacks;
        // Symbol: drake::solvers::MathematicalProgram::FindDecisionVariableIndex
        struct /* FindDecisionVariableIndex */ {
          // Source: drake/solvers/mathematical_program.h:2547
          const char* doc =
R"""(Returns the index of the decision variable. Internally the solvers
thinks all variables are stored in an array, and it accesses each
individual variable using its index. This index is used when adding
constraints and costs for each solver.

Precondition:
    {``var`` is a decision variable in the mathematical program,
    otherwise this function throws a runtime error.})""";
        } FindDecisionVariableIndex;
        // Symbol: drake::solvers::MathematicalProgram::FindDecisionVariableIndices
        struct /* FindDecisionVariableIndices */ {
          // Source: drake/solvers/mathematical_program.h:2558
          const char* doc =
R"""(Returns the indices of the decision variables. Internally the solvers
thinks all variables are stored in an array, and it accesses each
individual variable using its index. This index is used when adding
constraints and costs for each solver.

Precondition:
    {``vars`` are decision variables in the mathematical program,
    otherwise this function throws a runtime error.})""";
        } FindDecisionVariableIndices;
        // Symbol: drake::solvers::MathematicalProgram::FindIndeterminateIndex
        struct /* FindIndeterminateIndex */ {
          // Source: drake/solvers/mathematical_program.h:2571
          const char* doc =
R"""(Returns the index of the indeterminate. Internally a solver thinks all
indeterminates are stored in an array, and it accesses each individual
indeterminate using its index. This index is used when adding
constraints and costs for each solver.

Precondition:
    ``var`` is a indeterminate in the mathematical program, otherwise
    this function throws a runtime error.)""";
        } FindIndeterminateIndex;
        // Symbol: drake::solvers::MathematicalProgram::GetAllConstraints
        struct /* GetAllConstraints */ {
          // Source: drake/solvers/mathematical_program.h:2507
          const char* doc =
R"""(Getter for returning all constraints.

Returns:
    Vector of all constraint bindings.

Note:
    The group ordering may change as more constraint types are added.)""";
        } GetAllConstraints;
        // Symbol: drake::solvers::MathematicalProgram::GetAllCosts
        struct /* GetAllCosts */ {
          // Source: drake/solvers/mathematical_program.h:2482
          const char* doc =
R"""(Getter returning all costs.

Returns:
    Vector of all cost bindings.

Note:
    The group ordering may change as more cost types are added.)""";
        } GetAllCosts;
        // Symbol: drake::solvers::MathematicalProgram::GetAllLinearConstraints
        struct /* GetAllLinearConstraints */ {
          // Source: drake/solvers/mathematical_program.h:2495
          const char* doc =
R"""(Getter returning all linear constraints (both linear equality and
inequality constraints).

Returns:
    Vector of all linear constraint bindings.)""";
        } GetAllLinearConstraints;
        // Symbol: drake::solvers::MathematicalProgram::GetInitialGuess
        struct /* GetInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h:2265
          const char* doc_1args_decision_variable =
R"""(Gets the initial guess for a single variable.

Precondition:
    ``decision_variable`` has been registered in the optimization
    program.

Raises:
    RuntimeError if the pre condition is not satisfied.)""";
          // Source: drake/solvers/mathematical_program.h:2278
          const char* doc_1args_constEigenMatrixBase =
R"""(Gets the initial guess for some variables.

Precondition:
    Each variable in ``decision_variable_mat`` has been registered in
    the optimization program.

Raises:
    RuntimeError if the pre condition is not satisfied.)""";
        } GetInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::GetSolverOptionsDouble
        struct /* GetSolverOptionsDouble */ {
          // Source: drake/solvers/mathematical_program.h:2387
          const char* doc = R"""()""";
        } GetSolverOptionsDouble;
        // Symbol: drake::solvers::MathematicalProgram::GetSolverOptionsInt
        struct /* GetSolverOptionsInt */ {
          // Source: drake/solvers/mathematical_program.h:2392
          const char* doc = R"""()""";
        } GetSolverOptionsInt;
        // Symbol: drake::solvers::MathematicalProgram::GetSolverOptionsStr
        struct /* GetSolverOptionsStr */ {
          // Source: drake/solvers/mathematical_program.h:2397
          const char* doc = R"""()""";
        } GetSolverOptionsStr;
        // Symbol: drake::solvers::MathematicalProgram::MakeCost
        struct /* MakeCost */ {
          // Source: drake/solvers/mathematical_program.h:754
          const char* doc =
R"""(Convert an input of type ``F`` to a FunctionCost object.

Template parameter ``F``:
    This class should have functions numInputs(), numOutputs and
    eval(x, y).)""";
        } MakeCost;
        // Symbol: drake::solvers::MathematicalProgram::MathematicalProgram
        struct /* ctor */ {
          // Source: drake/solvers/mathematical_program.h:142
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::solvers::MathematicalProgram::NewBinaryVariables
        struct /* NewBinaryVariables */ {
          // Source: drake/solvers/mathematical_program.h:303
          const char* doc_3args =
R"""(Adds binary variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Template parameter ``Rows``:
    The number of rows in the new variables.

Template parameter ``Cols``:
    The number of columns in the new variables.

Parameter ``rows``:
    The number of rows in the new variables.

Parameter ``cols``:
    The number of columns in the new variables.

Parameter ``name``:
    The commonly shared name of the new variables.

Returns:
    The MatrixDecisionVariable of size rows x cols, containing the new
    vars (not all the vars stored).

Example:


::

    MathematicalProgram prog;
    auto b = prog.NewBinaryVariables(2, 3, "b");

This adds a 2 x 3 matrix decision variables into the program.

The name of the variable is only used for the user in order to ease
readability.)""";
          // Source: drake/solvers/mathematical_program.h:323
          const char* doc_1args =
R"""(Adds a matrix of binary variables into the optimization program.

Template parameter ``Rows``:
    The number of rows in the newly added binary variables.

Template parameter ``Cols``:
    The number of columns in the new variables. The default is 1.

Parameter ``name``:
    Each newly added binary variable will share the same name. The
    default name is "b".

Returns:
    A matrix containing the newly added variables.)""";
          // Source: drake/solvers/mathematical_program.h:334
          const char* doc_2args =
R"""(Adds binary variables to this MathematicalProgram. The new variables
are viewed as a column vector, with size ``rows`` x 1.

See also:
    NewBinaryVariables(int rows, int cols, const
    std::vector<std::string>& names);)""";
        } NewBinaryVariables;
        // Symbol: drake::solvers::MathematicalProgram::NewContinuousVariables
        struct /* NewContinuousVariables */ {
          // Source: drake/solvers/mathematical_program.h:196
          const char* doc_2args =
R"""(Adds continuous variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Parameter ``rows``:
    The number of rows in the new variables.

Parameter ``name``:
    The name of the newly added variables

Returns:
    The VectorDecisionVariable of size rows x 1, containing the new
    vars (not all the vars stored).

Example:


::

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(2, "x");

This adds a 2 x 1 vector containing decision variables into the
program. The names of the variables are "x(0)" and "x(1)".

The name of the variable is only used for the user in order to ease
readability.)""";
          // Source: drake/solvers/mathematical_program.h:233
          const char* doc_3args =
R"""(Adds continuous variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Template parameter ``Rows``:
    The number of rows of the new variables, in the compile time.

Template parameter ``Cols``:
    The number of columns of the new variables, in the compile time.

Parameter ``rows``:
    The number of rows in the new variables. When Rows is not
    Eigen::Dynamic, rows is ignored.

Parameter ``cols``:
    The number of columns in the new variables. When Cols is not
    Eigen::Dynamic, cols is ignored.

Parameter ``name``:
    All variables will share the same name, but different index.

Returns:
    The MatrixDecisionVariable of size Rows x Cols, containing the new
    vars (not all the vars stored).

Example:


::

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(2, 3, "X");
    auto y = prog.NewContinuousVariables<2, 3>(2, 3, "X");

This adds a 2 x 3 matrix decision variables into the program.

The name of the variable is only used for the user in order to ease
readability.)""";
          // Source: drake/solvers/mathematical_program.h:270
          const char* doc_1args =
R"""(Adds continuous variables, appending them to an internal vector of any
existing vars. The initial guess values for the new variables are set
to NaN, to indicate that an initial guess has not been assigned.
Callers are expected to add costs and/or constraints to have any
effect during optimization. Callers can also set the initial guess of
the decision variables through SetInitialGuess() or
SetInitialGuessForAllVariables().

Template parameter ``Rows``:
    The number of rows in the new variables.

Template parameter ``Cols``:
    The number of columns in the new variables. The default is 1.

Parameter ``name``:
    All variables will share the same name, but different index.

Returns:
    The MatrixDecisionVariable of size rows x cols, containing the new
    vars (not all the vars stored).

Example:


::

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables<2, 3>("X");

This adds a 2 x 3 matrix decision variables into the program.

The name of the variable is only used for the user in order to ease
readability.)""";
        } NewContinuousVariables;
        // Symbol: drake::solvers::MathematicalProgram::NewFreePolynomial
        struct /* NewFreePolynomial */ {
          // Source: drake/solvers/mathematical_program.h:409
          const char* doc =
R"""(Returns a free polynomial in a monomial basis over ``indeterminates``
of a given ``degree``. It uses ``coeff_name`` to make new decision
variables and use them as coefficients. For example,
``NewFreePolynomial({x₀, x₁}, 2)`` returns a₀x₁² + a₁x₀x₁ + a₂x₀² +
a₃x₁ + a₄x₀ + a₅.)""";
        } NewFreePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewIndeterminates
        struct /* NewIndeterminates */ {
          // Source: drake/solvers/mathematical_program.h:628
          const char* doc_2args =
R"""(Adds indeterminates to this MathematicalProgram, with default name
"x".

See also:
    NewIndeterminates(int rows, int cols, const
    std::vector<std::string>& names);)""";
          // Source: drake/solvers/mathematical_program.h:663
          const char* doc_3args =
R"""(Adds indeterminates to this MathematicalProgram, with default name
"X". The new variables are returned and viewed as a matrix, with size
``rows`` x ``cols``.

See also:
    NewIndeterminates(int rows, int cols, const
    std::vector<std::string>& names);)""";
        } NewIndeterminates;
        // Symbol: drake::solvers::MathematicalProgram::NewNonnegativePolynomial
        struct /* NewNonnegativePolynomial */ {
          // Source: drake/solvers/mathematical_program.h:442
          const char* doc_2args_monomial_basis_type =
R"""(Returns a pair of nonnegative polynomial p = mᵀQm and the Grammian
matrix Q, where m is ``monomial_basis``. Adds Q as decision variables
to the program. Depending on the type of the polynomial, we will
impose different constraint on Q. - if type = kSos, we impose Q being
positive semidefinite. - if type = kSdsos, we impose Q being scaled
diagonally dominant. - if type = kDsos, we impose Q being positive
diagonally dominant.

Parameter ``monomial_basis``:
    The monomial basis.

Parameter ``type``:
    The type of the nonnegative polynomial.

Returns:
    (p, Q) The polynomial p and the Grammian matrix Q. Q has been
    added as decision variables to the program.)""";
          // Source: drake/solvers/mathematical_program.h:450
          const char* doc_3args_grammian_monomial_basis_type =
R"""(Overloads NewNonnegativePolynomial(), except the Grammian matrix Q is
an input instead of an output.)""";
          // Source: drake/solvers/mathematical_program.h:470
          const char* doc_3args_indeterminates_degree_type =
R"""(Overloads NewNonnegativePolynomial(). Instead of passing the monomial
basis, we use a monomial basis that contains all monomials of
``indeterminates`` of total order up to ``degree`` / 2, hence the
returned polynomial p contains all the monomials of ``indeterminates``
of total order up to ``degree``.

Parameter ``indeterminates``:
    All the indeterminates in the polynomial p.

Parameter ``degree``:
    The polynomial p will contain all the monomials up to order
    ``degree``.

Parameter ``type``:
    The type of the nonnegative polynomial.

Returns:
    (p, Q) The polynomial p and the Grammian matrix Q. Q has been
    added as decision variables to the program.

Precondition:
    ``degree`` is a positive even number.)""";
        } NewNonnegativePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewSosPolynomial
        struct /* NewSosPolynomial */ {
          // Source: drake/solvers/mathematical_program.h:482
          const char* doc_1args =
R"""(Returns a pair of a SOS polynomial p = mᵀQm and the Grammian matrix Q,
where m is the ``monomial`` basis. For example,
``NewSosPolynomial(Vector2<Monomial>{x,y})`` returns a polynomial p =
Q₍₀,₀₎x² + 2Q₍₁,₀₎xy + Q₍₁,₁₎y² and Q.

Note:
    Q is a symmetric monomial_basis.rows() x monomial_basis.rows()
    matrix.)""";
          // Source: drake/solvers/mathematical_program.h:495
          const char* doc_2args =
R"""(Returns a pair of a SOS polynomial p = m(x)ᵀQm(x) of degree ``degree``
and the Grammian matrix Q that should be PSD, where m(x) is the result
of calling ``MonomialBasis(indeterminates, degree/2)``. For example,
``NewSosPolynomial({x}, 4)`` returns a pair of a polynomial p =
Q₍₀,₀₎x⁴ + 2Q₍₁,₀₎ x³ + (2Q₍₂,₀₎ + Q₍₁,₁₎)x² + 2Q₍₂,₁₎x + Q₍₂,₂₎ and
Q.

Raises:
    RuntimeError if ``degree`` is not a positive even integer.

See also:
    MonomialBasis.)""";
        } NewSosPolynomial;
        // Symbol: drake::solvers::MathematicalProgram::NewSymmetricContinuousVariables
        struct /* NewSymmetricContinuousVariables */ {
          // Source: drake/solvers/mathematical_program.h:357
          const char* doc_2args =
R"""(Adds a runtime sized symmetric matrix as decision variables to this
MathematicalProgram. The optimization will only use the stacked
columns of the lower triangular part of the symmetric matrix as
decision variables.

Parameter ``rows``:
    The number of rows in the symmetric matrix.

Parameter ``name``:
    The name of the matrix. It is only used the for user to understand
    the optimization program. The default name is "Symmetric", and
    each variable will be named as


::

    Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
    Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
               ...
    Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)

Notice that the (i,j)'th entry and (j,i)'th entry has the same name.

Returns:
    The newly added decision variables.)""";
          // Source: drake/solvers/mathematical_program.h:379
          const char* doc_1args =
R"""(Adds a static sized symmetric matrix as decision variables to this
MathematicalProgram. The optimization will only use the stacked
columns of the lower triangular part of the symmetric matrix as
decision variables.

Template parameter ``rows``:
    The number of rows in the symmetric matrix.

Parameter ``name``:
    The name of the matrix. It is only used the for user to understand
    the optimization program. The default name is "Symmetric", and
    each variable will be named as


::

    Symmetric(0, 0)     Symmetric(1, 0)     ... Symmetric(rows-1, 0)
    Symmetric(1, 0)     Symmetric(1, 1)     ... Symmetric(rows-1, 1)
               ...
    Symmetric(rows-1,0) Symmetric(rows-1,1) ... Symmetric(rows-1, rows-1)

Notice that the (i,j)'th entry and (j,i)'th entry has the same name.

Returns:
    The newly added decision variables.)""";
        } NewSymmetricContinuousVariables;
        // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial
        struct /* NonnegativePolynomial */ {
          // Source: drake/solvers/mathematical_program.h:422
          const char* doc =
R"""(Types of non-negative polynomial that can be found through conic
optimization. We currently support SOS, SDSOS and DSOS. For more
information about these polynomial types, please refer to "DSOS and
SDSOS Optimization: More Tractable Alternatives to Sum of Squares and
Semidefinite Optimization" by Amir Ali Ahmadi and Anirudha Majumdar,
with arXiv link https://arxiv.org/abs/1706.02586)""";
          // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial::kDsos
          struct /* kDsos */ {
            // Source: drake/solvers/mathematical_program.h:425
            const char* doc =
R"""(< A diagonally dominant sum-of-squares polynomial.)""";
          } kDsos;
          // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial::kSdsos
          struct /* kSdsos */ {
            // Source: drake/solvers/mathematical_program.h:424
            const char* doc =
R"""(< A scaled-diagonally dominant sum-of-squares polynomial.)""";
          } kSdsos;
          // Symbol: drake::solvers::MathematicalProgram::NonnegativePolynomial::kSos
          struct /* kSos */ {
            // Source: drake/solvers/mathematical_program.h:423
            const char* doc = R"""(< A sum-of-squares polynomial.)""";
          } kSos;
        } NonnegativePolynomial;
        // Symbol: drake::solvers::MathematicalProgram::SetDecisionVariableValueInVector
        struct /* SetDecisionVariableValueInVector */ {
          // Source: drake/solvers/mathematical_program.h:2339
          const char* doc_3args_decision_variable_decision_variable_new_value_values =
R"""(Updates the value of a single ``decision_variable`` inside the
``values`` vector to be ``decision_variable_new_value``. The other
decision variables' values in ``values`` are unchanged.

Parameter ``decision_variable``:
    a registered decision variable in this program.

Parameter ``decision_variable_new_value``:
    the variable's new values.

Parameter ``values``:
    The vector to be tweaked; must be of size num_vars().)""";
          // Source: drake/solvers/mathematical_program.h:2353
          const char* doc_3args_decision_variables_decision_variables_new_values_values =
R"""(Updates the values of some ``decision_variables`` inside the
``values`` vector to be ``decision_variables_new_values``. The other
decision variables' values in ``values`` are unchanged.

Parameter ``decision_variables``:
    registered decision variables in this program.

Parameter ``decision_variables_new_values``:
    the variables' respective new values; must have the same rows()
    and cols() sizes and ``decision_variables``.

Parameter ``values``:
    The vector to be tweaked; must be of size num_vars().)""";
        } SetDecisionVariableValueInVector;
        // Symbol: drake::solvers::MathematicalProgram::SetInitialGuess
        struct /* SetInitialGuess */ {
          // Source: drake/solvers/mathematical_program.h:2299
          const char* doc_2args_decision_variable_variable_guess_value =
R"""(Sets the initial guess for a single variable ``decision_variable``.
The guess is stored as part of this program.

Precondition:
    decision_variable is a registered decision variable in the
    program.

Raises:
    RuntimeError if precondition is not satisfied.)""";
          // Source: drake/solvers/mathematical_program.h:2308
          const char* doc_2args_constEigenMatrixBase_constEigenMatrixBase =
R"""(Sets the initial guess for the decision variables stored in
``decision_variable_mat`` to be ``x0``. The guess is stored as part of
this program.)""";
        } SetInitialGuess;
        // Symbol: drake::solvers::MathematicalProgram::SetInitialGuessForAllVariables
        struct /* SetInitialGuessForAllVariables */ {
          // Source: drake/solvers/mathematical_program.h:2326
          const char* doc =
R"""(Set the initial guess for ALL decision variables. Note that variables
begin with a default initial guess of NaN to indicate that no guess is
available.

Parameter ``x0``:
    A vector of appropriate size (num_vars() x 1).)""";
        } SetInitialGuessForAllVariables;
        // Symbol: drake::solvers::MathematicalProgram::SetSolverOption
        struct /* SetSolverOption */ {
          // Source: drake/solvers/mathematical_program.h:2358
          const char* doc = R"""()""";
        } SetSolverOption;
        // Symbol: drake::solvers::MathematicalProgram::SetSolverOptions
        struct /* SetSolverOptions */ {
          // Source: drake/solvers/mathematical_program.h:2378
          const char* doc =
R"""(Overwrite the stored solver options inside MathematicalProgram with
the provided solver options.)""";
        } SetSolverOptions;
        // Symbol: drake::solvers::MathematicalProgram::VarType
        struct /* VarType */ {
          // Source: drake/solvers/mathematical_program.h:143
          const char* doc = R"""()""";
        } VarType;
        // Symbol: drake::solvers::MathematicalProgram::bounding_box_constraints
        struct /* bounding_box_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2523
          const char* doc = R"""(Getter for all bounding box constraints)""";
        } bounding_box_constraints;
        // Symbol: drake::solvers::MathematicalProgram::decision_variable
        struct /* decision_variable */ {
          // Source: drake/solvers/mathematical_program.h:2685
          const char* doc =
R"""(Getter for the decision variable with index ``i`` in the program.)""";
        } decision_variable;
        // Symbol: drake::solvers::MathematicalProgram::decision_variable_index
        struct /* decision_variable_index */ {
          // Source: drake/solvers/mathematical_program.h:2708
          const char* doc =
R"""(Returns the mapping from a decision variable to its index in the
vector, containing all the decision variables in the optimization
program.)""";
        } decision_variable_index;
        // Symbol: drake::solvers::MathematicalProgram::decision_variables
        struct /* decision_variables */ {
          // Source: drake/solvers/mathematical_program.h:2680
          const char* doc =
R"""(Getter for all decision variables in the program.)""";
        } decision_variables;
        // Symbol: drake::solvers::MathematicalProgram::exponential_cone_constraints
        struct /* exponential_cone_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2473
          const char* doc =
R"""(Getter for exponential cone constraints.)""";
        } exponential_cone_constraints;
        // Symbol: drake::solvers::MathematicalProgram::generic_constraints
        struct /* generic_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2420
          const char* doc = R"""(Getter for all generic constraints)""";
        } generic_constraints;
        // Symbol: drake::solvers::MathematicalProgram::generic_costs
        struct /* generic_costs */ {
          // Source: drake/solvers/mathematical_program.h:2413
          const char* doc = R"""(Getter for all generic costs.)""";
        } generic_costs;
        // Symbol: drake::solvers::MathematicalProgram::indeterminate
        struct /* indeterminate */ {
          // Source: drake/solvers/mathematical_program.h:2693
          const char* doc =
R"""(Getter for the indeterminate with index ``i`` in the program.)""";
        } indeterminate;
        // Symbol: drake::solvers::MathematicalProgram::indeterminates
        struct /* indeterminates */ {
          // Source: drake/solvers/mathematical_program.h:2690
          const char* doc =
R"""(Getter for all indeterminates in the program.)""";
        } indeterminates;
        // Symbol: drake::solvers::MathematicalProgram::initial_guess
        struct /* initial_guess */ {
          // Source: drake/solvers/mathematical_program.h:2538
          const char* doc = R"""(Getter for the initial guess)""";
        } initial_guess;
        // Symbol: drake::solvers::MathematicalProgram::linear_complementarity_constraints
        struct /* linear_complementarity_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2530
          const char* doc =
R"""(Getter for all linear complementarity constraints.)""";
        } linear_complementarity_constraints;
        // Symbol: drake::solvers::MathematicalProgram::linear_constraints
        struct /* linear_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2443
          const char* doc = R"""(Getter for linear constraints.)""";
        } linear_constraints;
        // Symbol: drake::solvers::MathematicalProgram::linear_costs
        struct /* linear_costs */ {
          // Source: drake/solvers/mathematical_program.h:2433
          const char* doc = R"""(Getter for linear costs.)""";
        } linear_costs;
        // Symbol: drake::solvers::MathematicalProgram::linear_equality_constraints
        struct /* linear_equality_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2428
          const char* doc = R"""(Getter for linear equality constraints.)""";
        } linear_equality_constraints;
        // Symbol: drake::solvers::MathematicalProgram::linear_matrix_inequality_constraints
        struct /* linear_matrix_inequality_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2467
          const char* doc =
R"""(Getter for linear matrix inequality constraints.)""";
        } linear_matrix_inequality_constraints;
        // Symbol: drake::solvers::MathematicalProgram::lorentz_cone_constraints
        struct /* lorentz_cone_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2448
          const char* doc = R"""(Getter for Lorentz cone constraints.)""";
        } lorentz_cone_constraints;
        // Symbol: drake::solvers::MathematicalProgram::num_indeterminates
        struct /* num_indeterminates */ {
          // Source: drake/solvers/mathematical_program.h:2562
          const char* doc =
R"""(Gets the number of indeterminates in the optimization program)""";
        } num_indeterminates;
        // Symbol: drake::solvers::MathematicalProgram::num_vars
        struct /* num_vars */ {
          // Source: drake/solvers/mathematical_program.h:2535
          const char* doc =
R"""(Getter for number of variables in the optimization program)""";
        } num_vars;
        // Symbol: drake::solvers::MathematicalProgram::positive_semidefinite_constraints
        struct /* positive_semidefinite_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2461
          const char* doc =
R"""(Getter for positive semidefinite constraints.)""";
        } positive_semidefinite_constraints;
        // Symbol: drake::solvers::MathematicalProgram::quadratic_costs
        struct /* quadratic_costs */ {
          // Source: drake/solvers/mathematical_program.h:2438
          const char* doc = R"""(Getter for quadratic costs.)""";
        } quadratic_costs;
        // Symbol: drake::solvers::MathematicalProgram::required_capabilities
        struct /* required_capabilities */ {
          // Source: drake/solvers/mathematical_program.h:2699
          const char* doc =
R"""(Getter for the required capability on the solver, given the
cost/constraint/variable types in the program.)""";
        } required_capabilities;
        // Symbol: drake::solvers::MathematicalProgram::rotated_lorentz_cone_constraints
        struct /* rotated_lorentz_cone_constraints */ {
          // Source: drake/solvers/mathematical_program.h:2455
          const char* doc =
R"""(Getter for rotated Lorentz cone constraints.)""";
        } rotated_lorentz_cone_constraints;
        // Symbol: drake::solvers::MathematicalProgram::solver_options
        struct /* solver_options */ {
          // Source: drake/solvers/mathematical_program.h:2385
          const char* doc =
R"""(Returns the solver options stored inside MathematicalProgram.)""";
        } solver_options;
        // Symbol: drake::solvers::MathematicalProgram::visualization_callbacks
        struct /* visualization_callbacks */ {
          // Source: drake/solvers/mathematical_program.h:2405
          const char* doc = R"""(Getter for all callbacks.)""";
        } visualization_callbacks;
      } MathematicalProgram;
      // Symbol: drake::solvers::NewSymmetricVariableNames
      struct /* NewSymmetricVariableNames */ {
        // Source: drake/solvers/mathematical_program.h:64
        const char* doc = R"""()""";
      } NewSymmetricVariableNames;
      // Symbol: drake::solvers::NewVariableNames
      struct /* NewVariableNames */ {
        // Source: drake/solvers/mathematical_program.h:42
        const char* doc = R"""()""";
        // Symbol: drake::solvers::NewVariableNames::type
        struct /* type */ {
          // Source: drake/solvers/mathematical_program.h:56
          const char* doc = R"""()""";
        } type;
      } NewVariableNames;
    } solvers;
  } drake;
} pydrake_doc;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
