#include <algorithm>
#include <cmath>
#include <mutex>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>

namespace s1
{
// The SDF joints provide the actual constraint. This system only maps the
// existing body-frame Twist interface to the world-aligned slider joints.
class PlanarDrive : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    const gz::sim::Model model(_entity);
    this->base = model.LinkByName(_ecm, "base_link");
    this->xJoint = model.JointByName(_ecm, "ground_x");
    this->yJoint = model.JointByName(_ecm, "ground_y");
    this->yawJoint = model.JointByName(_ecm, "ground_yaw");
    for (const auto joint : {this->xJoint, this->yJoint, this->yawJoint})
    {
      if (joint && !_ecm.Component<gz::sim::components::JointVelocity>(joint))
        _ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
    }
    this->node.Subscribe("/model/" + model.Name(_ecm) + "/cmd_vel",
                        &PlanarDrive::OnCommand, this);
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->base || !this->xJoint ||
        !this->yJoint || !this->yawJoint)
      return;
    std::lock_guard<std::mutex> lock(this->mutex);
    const double yaw = gz::sim::worldPose(this->base, _ecm).Rot().Yaw();
    const double c = std::cos(yaw), s = std::sin(yaw);
    // Bounded forces preserve collision response. Setting joint velocities
    // directly would make an ideal motor push straight through obstacles.
    this->Drive(_ecm, this->xJoint, c * this->vx - s * this->vy, 25.0, 20.0);
    this->Drive(_ecm, this->yJoint, s * this->vx + c * this->vy, 25.0, 20.0);
    this->Drive(_ecm, this->yawJoint, this->wz, 1.0, 1.0);
  }

  private: void Drive(gz::sim::EntityComponentManager &_ecm,
      gz::sim::Entity joint, double target, double gain, double limit)
  {
    const auto velocity = _ecm.Component<gz::sim::components::JointVelocity>(joint);
    if (!velocity || velocity->Data().empty())
      return;
    const double force = std::clamp(gain * (target - velocity->Data()[0]), -limit, limit);
    _ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint, {force});
  }

  private: void OnCommand(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->vx = std::isfinite(_msg.linear().x()) ? _msg.linear().x() : 0;
    this->vy = std::isfinite(_msg.linear().y()) ? _msg.linear().y() : 0;
    this->wz = std::isfinite(_msg.angular().z()) ? _msg.angular().z() : 0;
  }

  private: gz::transport::Node node;
  private: std::mutex mutex;
  private: double vx{0}, vy{0}, wz{0};
  private: gz::sim::Entity base{gz::sim::kNullEntity};
  private: gz::sim::Entity xJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity yJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity yawJoint{gz::sim::kNullEntity};
};
}
GZ_ADD_PLUGIN(s1::PlanarDrive, gz::sim::System,
              s1::PlanarDrive::ISystemConfigure, s1::PlanarDrive::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(s1::PlanarDrive, "s1::PlanarDrive")
