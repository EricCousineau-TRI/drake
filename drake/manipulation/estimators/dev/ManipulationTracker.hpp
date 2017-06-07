#ifndef MANIPULATION_TRACKER_H
#define MANIPULATION_TRACKER_H

#include <stdexcept>
#include <iostream>

#include "drake/multibody/rigid_body_tree.h"

#include "costs/ManipulationTrackerCost.hpp"
#include "yaml-cpp/yaml.h"
#include <lcm/lcm-cpp.hpp>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/rigid_transform_t.hpp"

// forward def
class ManipulationTrackerCost;

typedef std::map<int, std::string> PlantIdMap;

std::shared_ptr<RigidBodyTreed> setupRobotFromConfig(
    YAML::Node config, Eigen::VectorXd& x0_robot, std::string base_path,
    PlantIdMap& plant_id_map, bool verbose = false, bool less_collision = false);
std::shared_ptr<RigidBodyTreed> setupRobotFromConfigSubset(YAML::Node config, Eigen::VectorXd& x0_robot_subset, std::string base_path,
    bool verbose, bool less_collision, bool exclusionary, std::vector<std::string> exceptions, std::vector<int> &index_correspondences);

class ManipulationTracker {
public:
  typedef std::pair<std::shared_ptr<ManipulationTrackerCost>, std::vector<int>> CostAndView;

  ManipulationTracker(std::shared_ptr<const RigidBodyTreed> robot,
                      Eigen::VectorXd x0_robot, std::shared_ptr<lcm::LCM> lcm,
                      YAML::Node config, const PlantIdMap& plant_id_map,
                      bool verbose = false);
  ~ManipulationTracker() {}

  void initBotConfig(const char* filename);
  int get_trans_with_utime(std::string from_frame, std::string to_frame,
                               long long utime, Eigen::Isometry3d & mat);

  bool has_performed_first_update() const { return has_performed_first_update_; }
  void set_q0(const Eigen::VectorXd& q0); // , const Eigen::VectorXd& v0);
  Eigen::Ref<const Eigen::VectorXd> get_q0() const;
  
  // register a cost function with the solver
  void addCost(std::shared_ptr<ManipulationTrackerCost> new_cost);

  void update();
  std::shared_ptr<const RigidBodyTreed> getRobot() { return robot_; }
  const PlantIdMap& getPlantIdMap() const { return plant_id_map_; }
  Eigen::VectorXd getMean() const { return x_; }
  Eigen::VectorXd getRobotQ() const;
  Eigen::MatrixXd getCovariance() { return covar_; }

  // helper to publish out to lcm
  void publish();

private:
  std::shared_ptr<const RigidBodyTreed> robot_;
  std::vector<std::string> robot_names_;
  KinematicsCache<double> robot_kinematics_cache_;

  struct publish_info {
    std::string robot_name;
    std::string publish_type;
    std::string publish_channel;
  };
  std::vector<publish_info> publish_infos_;

  bool do_post_transform_ = false;
  std::string post_transform_robot_;
  std::string post_transform_dest_frame_;
  bool do_force_align_ = false;
  std::string force_align_robot_;
  std::string force_align_dest_frame_;
//  BotParam* botparam_ = NULL;
//  BotFrames* botframes_ = NULL;

  Eigen::VectorXd x_;
  Eigen::MatrixXd covar_;

  std::shared_ptr<lcm::LCM> lcm_;

  // dynamics updating
  struct DynamicsVars {
      double floating_base_var = INFINITY;
      double other_var = INFINITY;
  };
  DynamicsVars dynamics_vars_defaults_;

  std::map<std::string, DynamicsVars> dynamics_vars_per_robot_;
  bool dynamics_verbose_ = false;

  // store all registered costs alongside a view into the variable
  // list, i.e. which decision vars that cost uses
  std::vector<CostAndView> registeredCostInfo_;

  bool verbose_;

  // whether update() has been called yet (to set IC)
  bool has_performed_first_update_{false};

  PlantIdMap plant_id_map_;
};

#endif
