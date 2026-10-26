// Effective measured drive response; fixed wheels and contact friction are model assumptions.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <cmath>

namespace intramural {
class Drive : public ignition::gazebo::System,
              public ignition::gazebo::ISystemConfigure,
              public ignition::gazebo::ISystemPreUpdate {
  ignition::gazebo::Link link;
  ignition::transport::Node node;
  std::mutex mutex;
  ignition::math::Vector3d command;
  std::deque<std::pair<double, ignition::math::Vector3d>> queue;
  double delay{.09}, tau{.06}, acceleration{3.5}, mass{1.};
public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    auto links = model.Links(ecm);
    if (links.empty()) throw std::runtime_error("Drive requires a link");
    link = ignition::gazebo::Link(model.CanonicalLink(ecm));
    mass = 0.;
    for (auto e : links) {
      auto inertial = ecm.Component<ignition::gazebo::components::Inertial>(e);
      if (inertial) mass += inertial->Data().MassMatrix().Mass();
    }
    delay = sdf->Get<double>("delay", .09).first;
    tau = sdf->Get<double>("tau", .06).first;
    acceleration = sdf->Get<double>("acceleration", 3.5).first;
    if (!std::isfinite(delay+tau+acceleration+mass) || delay<0 || tau<=0 || acceleration<=0 || mass<=0)
      throw std::runtime_error("Finite positive drive response required");
    node.Subscribe(sdf->Get<std::string>("topic"), &Drive::Command, this);
    link.EnableVelocityChecks(ecm);
  }
  void Command(const ignition::msgs::Twist &msg) {
    if (!std::isfinite(msg.linear().x()) || !std::isfinite(msg.linear().y())) return;
    std::lock_guard<std::mutex> guard(mutex);
    command.Set(msg.linear().x(), msg.linear().y(), 0.);
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                 ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused) return;
    double now = std::chrono::duration<double>(info.simTime).count();
    ignition::math::Vector3d requested;
    { std::lock_guard<std::mutex> guard(mutex); requested = command; }
    queue.emplace_back(now+delay, requested);
    while (queue.size()>1 && queue[1].first<=now) queue.pop_front();
    auto applied = queue.front().first<=now ? queue.front().second : ignition::math::Vector3d::Zero;
    auto pose = link.WorldPose(ecm);
    auto velocity = link.WorldLinearVelocity(ecm);
    if (!pose || !velocity) return;
    auto desired = pose->Rot().RotateVector(applied);
    auto a = (desired-*velocity)/tau;
    a.Z(0.);
    if (a.Length()>acceleration) a *= acceleration/a.Length();
    link.AddWorldForce(ecm, mass*a);
    // No command watchdog: transport-disconnect behaviour remains observable.
  }
};
}
IGNITION_ADD_PLUGIN(intramural::Drive, ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure, ignition::gazebo::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(intramural::Drive, "intramural::Drive")
