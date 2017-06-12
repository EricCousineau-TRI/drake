#pragma once

#include <drake/solvers/mathematical_program.h>

#include "drake/common/nice_type_name.h"
#include "drake/manipulation/estimators/dev/dart_util.h"

namespace drake {
namespace manipulation {

/**
 * A scene that DART is to be used for estimation.
 * Contains a scene model, the instances within the scene, and dictates
 * what joints are being optimized over vs. which ones are direct pass-through.
 */
// TODO(eric.cousineau): Consider merging with WorldSimBuilder.
class DartScene {
 public:
  DartScene(TreePtr tree,
            const InstanceIdMap& instance_id_map);

  int GetInstanceId(const string& name) const {
    return instance_id_map_.at(name);
  }
  const RigidBodyTreed& tree() const {
    return *tree_;
  }

  const vector<string>& position_names() const {
    return position_names_;
  }

  const vector<string>& velocity_names() const {
    return velocity_names_;
  }

  KinematicsSlice CreateKinematicsSlice(
      const vector<string>& sub_positions,
      const vector<string>& sub_velocities) const;

 private:
  TreePtr tree_;
  InstanceIdMap instance_id_map_;
  ReverseIdMap instance_name_map_;
  vector<string> position_names_;
  vector<string> velocity_names_;
};

class DartObjective;
typedef vector<unique_ptr<DartObjective>> DartObjectiveList;

/**
 * Contains information regarding formulation of tracking problem, such as the
 * objectives, the variables they create, etc. Will NOT store anything about
 * the state of the solution (e.g., initial condition (aside from the scene),
 * etc.).
 */
// TODO(eric.cousineau): Consider just placing equality constraints rather than
// removing the non-estimated joints.
class DartFormulation {
 public:
  struct Param {
    vector<string> estimated_positions;
    vector<string> estimated_velocities;
  };

  DartFormulation(unique_ptr<DartScene> scene, const Param& param);

  /**
   * Create and add an objective, setting this as the formulation.
   */
  template <typename Obj, typename ... Args>
  Obj* AddObjective(Args&&... args) {
    Obj* objective = new Obj(this, std::forward<Args>(args)...);
    AddObjective(CreateUnique(objective));
  }

  /**
   * Add and initialize an objective for this formulation.
   */
  void AddObjective(unique_ptr<DartObjective> objective);

  void Compile() {

    // To be called by DartEstimator. Get slices for each objective.
    for (auto& objective : objectives()) {

    }
  }

  const KinematicsSlice& kinematics_slice() const { return kinematics_slice_; }
  const VectorSlice& q_slice() const { return kinematics_slice_.q(); }

  const DartObjectiveList& objectives() const { return objectives_; }
  const VectorSlice& objective_var_slice(int i) const {
    return objective_var_slices_[i];
  }

  /**
   * This is a VERY relaxed interface for objectives to access.
   * They should NOT attempt to call solve.
   * Additionally, no attempt should be made to obtain the prior solution state.
   * Rather, this will be passed in via DartObjective::Update().
   */
  MathematicalProgram& prog() { return prog_; }
  const RigidBodyTreed& tree() const { return scene_->tree(); }
private:
  Param param_;
  unique_ptr<DartScene> scene_;
  MathematicalProgram prog_;
  KinematicsVars kinematics_vars_;
  KinematicsSlice kinematics_slice_;

  DartObjectiveList objectives_;
  // Track which values are use for the optimization.
  KinematicsSlice kinematics_var_slice_;
  vector<VectorSlice> objective_var_slices_;
};

/**
 * An QP EKF objective may have induce both costs and constraints.
 * The objective will NOT produce decision variables for the independent
 * kinematic variables (position, velocity).
 * Rather, it will produce any additional slack variables it needs, and
 * store its own state internally.
 */
// TODO(eric.cousineau): Look into using a System-compatible but not System-
// required storage mode for state. (do not worry about inputs).
// Refer to Alejandro's MultiBodyTree formulation, and usage of a System-like
// context and cache.
class DartObjective {
 public:
  // TODO(eric.cousineau): See if there is a way to remove this.
  static constexpr double kInitialUncorrelatedVariance = 1e-10;

  DartObjective(DartFormulation* formulation)
      : formulation_(formulation) {}

  /**
   * Initialize the objective, adding costs and/or constraints.
   */
  virtual void Init() = 0;

  /// Get optimization variables for given objective.
  virtual const OptVars& GetVars() const = 0;

  /// Get initial values for above values.
  virtual const VectorXd& GetInitialValues() const = 0;

  // TODO(eric.cousineau): Figure out theoretically succinct initialization.
  virtual MatrixXd GetInitialCovariance() const {
    // TODO(eric.cousineau): See if there is a better way to do this, per
    // Greg's comments.
    const int num_vars = GetVars().size();
    return kInitialUncorrelatedVariance *
        MatrixXd::Identity(num_vars, num_vars);
  }


  virtual bool RequiresObservation() const {
    return true;
  }

  virtual void UpdateFormulation(
      double t,
      const KinematicsCached* cache_prior,
      const VectorXd& obj_priors) = 0;


  string name() const {
    return NiceTypeName::Get(*this);
  }

  /**
   * Each individual subclass must implement its own Observe() functionality
   * to take in (a) time and (b) measurement data (joint states, image, etc.).
   */
  double latest_observation_time() const {
    return latest_observation_time_;
  }

 protected:
  void set_latest_observation_time(double time) {
    latest_observation_time_ = time;
  }
  DartFormulation& formulation() const { return *formulation_; }
 private:
  DartFormulation* formulation_{};
  double latest_observation_time_{};
};

class DartJointObjective : public DartObjective {
 public:
  void Observe(const KinematicsState& full_state) {
  }
 private:
};

// TODO(eric.cousinaeu): Consider formulation necessary for multi-rate
// esimators, such as in Cifuentes et al. - note that this is a particle
// filter, and may not fit into this framework at all.
// TODO(eric.cousineau): Consider formulation for adding / removing bodies,
// where the number of variables needs to change on the fly.
class DartEstimator {
 public:
  struct Param {
    // This MUST be initialized.
    KinematicsState initial_state;
    double max_observed_time_diff {1e-4};
  };

  DartEstimator(unique_ptr<DartFormulation> formulation, const Param& param)
    : param_(param),
      formulation_(std::move(formulation)),
      state_prior_(formulation->tree()),
      cache_prior_(formulation_->tree().CreateKinematicsCache()) {
    // Add kinematics variables.
  }

  /**
   * Ensure that the optimization problem is ready to be performed.
   */
  void Compile() {
    // Aggregate covariance.
    int num_var = prog().num_vars();
    covariance_.resize(num_var, num_var);
    covariance_.setZero();
    for (auto& objective : objectives()) {
      // Aggregate covariance.
    }
  }

  const KinematicsState& Update(double t);

  const KinematicsState& state_prior() const { return state_prior_; }

 protected:
  // Convenience.
  const RigidBodyTreed& tree() const { return formulation_->tree(); }
  MathematicalProgram& prog() { return formulation_->prog(); }
  const DartObjectiveList& objectives() { return formulation_->objectives(); }
 private:
  Param param_;
  unique_ptr<DartFormulation> formulation_;

  MatrixXd covariance_;
  VectorXd opt_var_prior_;

  KinematicsState state_prior_;
  KinematicsCached cache_prior_;
};

}  // manipulation
}  // drake
