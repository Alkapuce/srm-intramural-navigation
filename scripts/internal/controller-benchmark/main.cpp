// Offline JSON-line boundary around the installed Nav2 optimizer and locked PB plugin.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_mppi_controller/optimizer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using json = nlohmann::json;
using Vec = std::array<double, 2>;

Vec limited(Vec v, double limit) {
  const double scale = std::min(1., limit / std::max(1e-9, std::hypot(v[0], v[1])));
  return {v[0] * scale, v[1] * scale};
}

class NoYawOmni : public mppi::OmniMotionModel {
public:
  void predict(mppi::models::State & s) override {
    s.cwz.fill(0.);mppi::OmniMotionModel::predict(s);s.wz.fill(0.);
  }
};
class ResponseModel : public NoYawOmni {
public:
  ResponseModel(const json & c) : dt(c["dt"]), delay(c["model_delay"]), tau(c["model_tau"]),
    speed(c["max_speed"]), acceleration(c["max_acceleration"]) {}
  double dt, delay, tau, speed, acceleration;
  Vec last{0., 0.};
  std::deque<Vec> history;

  void predict(mppi::models::State & s) override {
    const size_t lag = std::lround(delay / dt);
    const double alpha = 1. - std::exp(-dt / std::max(tau, 1e-6));
    for (size_t b = 0; b < s.vx.shape(0); ++b) {
      Vec previous = last;
      std::vector<Vec> controls(s.cvx.shape(1));
      for (size_t k = 0; k < s.cvx.shape(1); ++k) {
        Vec desired = limited({s.cvx(b,k), s.cvy(b,k)}, speed);
        Vec delta = limited({desired[0]-previous[0], desired[1]-previous[1]}, acceleration*dt);
        previous = {previous[0]+delta[0], previous[1]+delta[1]};
        controls[k] = previous;s.cwz(b,k) = 0.;
        if (k + 1 < s.vx.shape(1)) {
          Vec applied = previous;
          if (k < lag) {
            const long offset = static_cast<long>(history.size()) - static_cast<long>(lag) + static_cast<long>(k);
            applied = offset >= 0 ? history[offset] : Vec{0.,0.};
          } else applied = controls[k-lag];
          s.vx(b,k+1) = s.vx(b,k) + alpha * (applied[0]-s.vx(b,k));
          s.vy(b,k+1) = s.vy(b,k) + alpha * (applied[1]-s.vy(b,k));
          s.wz(b,k+1) = 0.;
        }
      }
      s.wz(b,0) = 0.;
    }
  }
  void applyConstraints(mppi::models::ControlSequence & sequence) override {
    Vec previous = last;
    for (size_t k=0; k<sequence.vx.size(); ++k) {
      Vec desired = limited({sequence.vx(k),sequence.vy(k)},speed);
      Vec delta = limited({desired[0]-previous[0],desired[1]-previous[1]},acceleration*dt);
      previous = {previous[0]+delta[0],previous[1]+delta[1]};
      sequence.vx(k)=previous[0]; sequence.vy(k)=previous[1]; sequence.wz(k)=0.;
    }
  }
};
class OfflineOptimizer : public mppi::Optimizer {
public:
  void use(const std::shared_ptr<mppi::MotionModel> & model) { motion_model_ = model; }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::get_logger("controller_benchmark").set_level(rclcpp::Logger::Level::Error);
  int result = 0;
  try {
    std::string line;
    if (!std::getline(std::cin,line)) throw std::runtime_error("Missing benchmark configuration");
    const json c = json::parse(line);
    const std::string backend = c["backend"];
    if (backend != "pb" && backend != "mppi" && backend != "mppi-response")
      throw std::runtime_error("Unknown offline controller");
    const double dt=c["dt"], maximum=c["max_speed"];
    if (!(dt>0. && maximum>0.)) throw std::runtime_error("Invalid controller limits");
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_benchmark","/offline_comparison");
    node->declare_parameter("controller_frequency",1./dt);
    auto parameter=[&](const std::string & key, auto value) {node->declare_parameter("FollowPath."+key,value);};
    if (backend == "pb") {
      parameter("enable_rotation",false);parameter("use_rotate_to_heading",false);
      parameter("transform_tolerance",0.);
      parameter("translation_kp",4.);parameter("translation_ki",0.);
      parameter("translation_kd",.3);parameter("lookahead_dist",.18);
      parameter("use_velocity_scaled_lookahead_dist",false);parameter("min_lookahead_dist",.18);
      parameter("max_lookahead_dist",.18);parameter("v_linear_max",maximum);
      parameter("v_linear_min",-maximum);parameter("use_interpolation",true);
      parameter("min_approach_linear_velocity",.05);
      parameter("approach_velocity_scaling_dist",.2);parameter("reduction_ratio_at_high_curvature",.9);
    } else {
      parameter("motion_model",std::string("Omni"));parameter("model_dt",dt);
      parameter("time_steps",c.value("time_steps",80));parameter("batch_size",c.value("batch_size",512));
      parameter("iteration_count",1);parameter("vx_max",maximum);parameter("vx_min",-maximum);
      // Humble divides by sampling variance; constrain predicted yaw instead of using zero variance.
      parameter("vy_max",maximum);parameter("wz_max",0.);parameter("wz_std",.1);
      parameter("vx_std",.3);parameter("vy_std",.3);parameter("temperature",.3);
      parameter("critics",std::vector<std::string>{"ConstraintCritic","CostCritic","PathAlignCritic","PathFollowCritic","GoalCritic"});
      parameter("CostCritic.consider_footprint",false);parameter("CostCritic.cost_weight",3.81);
      parameter("CostCritic.near_goal_distance",.2);parameter("CostCritic.trajectory_point_step",1);
      parameter("PathAlignCritic.cost_weight",14.);parameter("PathAlignCritic.trajectory_point_step",2);
      parameter("PathAlignCritic.offset_from_furthest",4);parameter("PathAlignCritic.threshold_to_consider",.2);
      parameter("PathFollowCritic.cost_weight",5.);parameter("PathFollowCritic.offset_from_furthest",4);
      parameter("PathFollowCritic.threshold_to_consider",.2);parameter("GoalCritic.threshold_to_consider",.2);
    }
    auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>("comparison_costmap","/offline_comparison","costmap");
    costmap->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
    costmap->set_parameter(rclcpp::Parameter("global_frame","map"));
    costmap->set_parameter(rclcpp::Parameter("robot_base_frame","base_footprint"));
    costmap->set_parameter(rclcpp::Parameter("robot_radius",c.value("robot_radius",.29)));
    costmap->on_configure(rclcpp_lifecycle::State());
    auto grid=costmap->getCostmap();const auto & m=c["map"];
    grid->resizeMap(m["width"],m["height"],m["resolution"],m["origin"][0],m["origin"][1]);
    const auto cells=m["data"].get<std::vector<unsigned char>>();
    if (cells.size()!=grid->getSizeInCellsX()*grid->getSizeInCellsY()) throw std::runtime_error("Invalid grid size");
    std::copy(cells.begin(),cells.end(),grid->getCharMap());
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
    std::shared_ptr<nav2_core::Controller> pb;
    std::unique_ptr<OfflineOptimizer> optimizer;
    std::unique_ptr<mppi::ParametersHandler> handler;
    std::shared_ptr<ResponseModel> model;
    if (backend=="pb") {
      pb=loader.createSharedInstance("pb_omni_pid_pursuit_controller::OmniPidPursuitController");
      pb->configure(node,"FollowPath",tf,costmap);pb->activate();
    } else {
      handler=std::make_unique<mppi::ParametersHandler>(node);
      optimizer=std::make_unique<OfflineOptimizer>();optimizer->initialize(node,"FollowPath",costmap,handler.get());
      optimizer->use(std::make_shared<NoYawOmni>());
      if (backend=="mppi-response") {model=std::make_shared<ResponseModel>(c);optimizer->use(model);}
    }
    std::cout << json({{"ready",true},{"backend",backend}}).dump() << std::endl;
    while (std::getline(std::cin,line)) {
      const json request=json::parse(line);
      if (request.value("stop",false)) break;
      const auto started=std::chrono::steady_clock::now();
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.header.stamp=node->now();
      pose.pose.position.x=request["pose"][0];pose.pose.position.y=request["pose"][1];
      const double yaw=request["yaw"];tf2::Quaternion q;q.setRPY(0.,0.,yaw);pose.pose.orientation=tf2::toMsg(q);
      geometry_msgs::msg::TransformStamped transform;transform.header=pose.header;transform.child_frame_id="base_footprint";
      transform.transform.translation.x=pose.pose.position.x;transform.transform.translation.y=pose.pose.position.y;
      transform.transform.rotation=pose.pose.orientation;tf->setTransform(transform,"offline_comparison",true);
      geometry_msgs::msg::Twist velocity;velocity.linear.x=request["velocity"][0];velocity.linear.y=request["velocity"][1];
      nav_msgs::msg::Path path;path.header=pose.header;
      for (const auto & point:request["path"]) {auto p=pose;p.pose.position.x=point[0];p.pose.position.y=point[1];path.poses.push_back(p);}
      if (path.poses.size()<2) throw std::runtime_error("Path must contain at least two points");
      geometry_msgs::msg::Twist command;
      try {
        const double cap=request.value("speed_cap",maximum);
        if(model) {
          model->history.clear();
          for(const auto & u:request["pending_commands"]) model->history.push_back(u.get<Vec>());
          model->last=request.value("previous_command",Vec{0.,0.});model->speed=cap;
        }
        if(optimizer) optimizer->setSpeedLimit(cap,false);
        if(pb) {pb->setPlan(path);command=pb->computeVelocityCommands(pose,velocity,nullptr).twist;}
        else {command=optimizer->evalControl(pose,velocity,path,nullptr).twist;}
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
        if(!std::isfinite(command.linear.x)||!std::isfinite(command.linear.y)||std::abs(command.angular.z)>1e-9) throw std::runtime_error("Invalid or rotational command");
        std::cout << json({{"command",{command.linear.x,command.linear.y}},{"compute_ms",ms}}).dump()<<std::endl;
      } catch(const std::exception & e) {std::cout<<json({{"error",e.what()}}).dump()<<std::endl;}
    }
    if(pb) {pb->deactivate();pb->cleanup();pb.reset();}
    optimizer.reset();handler.reset();costmap->on_cleanup(rclcpp_lifecycle::State());costmap.reset();
  } catch(const std::exception & e) {std::cerr<<e.what()<<std::endl;result=1;}
  rclcpp::shutdown();return result;
}
