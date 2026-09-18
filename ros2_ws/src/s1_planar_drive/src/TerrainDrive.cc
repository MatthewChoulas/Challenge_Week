#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace s1
{
// Keep the rover on the DEM and reject motion onto slopes which are too steep.
class TerrainDrive : public gz::sim::System,
                     public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->base = gz::sim::Link(this->model.LinkByName(_ecm, "base_link"));
    this->base.EnableVelocityChecks(_ecm);
    this->base.SetGravityEnabled(_ecm, false);
    this->model.SetCollisionEnabled(_ecm, false);

    this->terrainSize = _sdf->Get<double>("terrain_size", 2000.0).first;
    this->heightRange = _sdf->Get<double>("height_range", 1.0).first;
    this->zOffset = _sdf->Get<double>("terrain_z_offset", 0.0).first;
    const double slopeDegrees = _sdf->Get<double>("max_slope_deg", 30.0).first;
    this->minNormalZ = std::cos(slopeDegrees * kPi / 180.0);

    const auto path = _sdf->Get<std::string>("heightmap_uri", "").first;
    if (path.empty() || this->heightmap.Load(path) != 0 ||
        !this->heightmap.Valid() || this->heightmap.Width() < 2 ||
        this->heightmap.Height() < 2)
    {
      gzerr << "TerrainDrive could not load heightmap [" << path << "]\n";
      return;
    }

    this->ready = true;
    this->node.Subscribe("/model/" + this->model.Name(_ecm) + "/cmd_vel",
                         &TerrainDrive::OnCommand, this);
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->ready || !this->base.Valid(_ecm))
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;

    double vx = 0.0;
    double vy = 0.0;
    double turn = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (std::chrono::steady_clock::now() - this->lastCommand <
          std::chrono::milliseconds(500))
      {
        vx = this->vx;
        vy = this->vy;
        turn = this->wz;
      }
    }

    const auto current = gz::sim::worldPose(this->base.Entity(), _ecm);
    double x = current.X();
    double y = current.Y();
    const double yaw = current.Rot().Yaw() + turn * dt;
    const double worldVx = std::cos(yaw) * vx - std::sin(yaw) * vy;
    const double worldVy = std::sin(yaw) * vx + std::cos(yaw) * vy;

    // Check intermediate positions so a 100 m/s command cannot jump over a
    // narrow impassable section during a delayed simulation update.
    const double travel = std::hypot(worldVx, worldVy) * dt;
    const int steps = std::max(1, static_cast<int>(std::ceil(travel / 0.20)));
    bool blocked = false;
    for (int step = 0; step < steps; ++step)
    {
      const double nextX = x + worldVx * dt / steps;
      const double nextY = y + worldVy * dt / steps;
      if (!this->Traversable(nextX, nextY, yaw))
      {
        blocked = true;
        break;
      }
      x = nextX;
      y = nextY;
    }

    gz::math::Pose3d pose;
    if (!this->SurfacePose(x, y, yaw, pose))
      return;
    this->model.SetWorldPoseCmd(_ecm, pose);
    this->base.SetLinearVelocity(
        _ecm, blocked ? gz::math::Vector3d::Zero :
        gz::math::Vector3d(vx, vy, 0.0));
    this->base.SetAngularVelocity(_ecm, gz::math::Vector3d(0.0, 0.0, turn));
  }

  private: double Height(double _x, double _y) const
  {
    const double half = this->terrainSize * 0.5;
    const double px = std::clamp((_x + half) / this->terrainSize *
        (this->heightmap.Width() - 1), 0.0,
        static_cast<double>(this->heightmap.Width() - 1));
    // Gazebo heightmaps map increasing image rows to increasing world Y.
    const double py = std::clamp((_y + half) / this->terrainSize *
        (this->heightmap.Height() - 1), 0.0,
        static_cast<double>(this->heightmap.Height() - 1));
    const auto x0 = static_cast<unsigned int>(std::floor(px));
    const auto y0 = static_cast<unsigned int>(std::floor(py));
    const auto x1 = std::min(x0 + 1, this->heightmap.Width() - 1);
    const auto y1 = std::min(y0 + 1, this->heightmap.Height() - 1);
    const double tx = px - x0;
    const double ty = py - y0;
    const double top = (1.0 - tx) * this->heightmap.Pixel(x0, y0).R() +
                       tx * this->heightmap.Pixel(x1, y0).R();
    const double bottom = (1.0 - tx) * this->heightmap.Pixel(x0, y1).R() +
                          tx * this->heightmap.Pixel(x1, y1).R();
    return this->zOffset + this->heightRange *
        ((1.0 - ty) * top + ty * bottom);
  }

  private: bool HeightAndNormal(double _x, double _y, double &_height,
      gz::math::Vector3d &_normal) const
  {
    constexpr double sample = 1.0;
    const double half = this->terrainSize * 0.5;
    if (_x < -half + sample || _x > half - sample ||
        _y < -half + sample || _y > half - sample)
      return false;
    _height = this->Height(_x, _y);
    const double dzdx = (this->Height(_x + sample, _y) -
                         this->Height(_x - sample, _y)) / (2.0 * sample);
    const double dzdy = (this->Height(_x, _y + sample) -
                         this->Height(_x, _y - sample)) / (2.0 * sample);
    _normal.Set(-dzdx, -dzdy, 1.0);
    _normal.Normalize();
    return true;
  }

  private: bool Traversable(double _x, double _y, double _yaw) const
  {
    const double c = std::cos(_yaw);
    const double s = std::sin(_yaw);
    for (const auto &wheel : kWheelOffsets)
    {
      double height;
      gz::math::Vector3d normal;
      const double wx = _x + c * wheel[0] - s * wheel[1];
      const double wy = _y + s * wheel[0] + c * wheel[1];
      if (!this->HeightAndNormal(wx, wy, height, normal) ||
          normal.Z() < this->minNormalZ)
        return false;
    }
    return true;
  }

  private: bool SurfacePose(double _x, double _y, double _yaw,
      gz::math::Pose3d &_pose) const
  {
    double centerHeight;
    gz::math::Vector3d normal;
    if (!this->HeightAndNormal(_x, _y, centerHeight, normal))
      return false;

    const double c = std::cos(_yaw);
    const double s = std::sin(_yaw);
    const double nx = c * normal.X() + s * normal.Y();
    const double ny = -s * normal.X() + c * normal.Y();
    const double roll = std::asin(std::clamp(-ny, -1.0, 1.0));
    const double pitch = std::atan2(nx, normal.Z());
    const gz::math::Quaterniond rotation(roll, pitch, _yaw);

    double bodyZ = centerHeight + kWheelCenterDepth + kWheelRadius;
    for (const auto &wheel : kWheelOffsets)
    {
      const auto offset = rotation.RotateVector(
          gz::math::Vector3d(wheel[0], wheel[1], -kWheelCenterDepth));
      bodyZ = std::max(bodyZ,
          this->Height(_x + offset.X(), _y + offset.Y()) +
          kWheelRadius - offset.Z());
    }
    _pose.Set(_x, _y, bodyZ, roll, pitch, _yaw);
    return true;
  }

  private: void OnCommand(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->vx = std::isfinite(_msg.linear().x()) ? _msg.linear().x() : 0.0;
    this->vy = std::isfinite(_msg.linear().y()) ? _msg.linear().y() : 0.0;
    const double speed = std::hypot(this->vx, this->vy);
    if (speed > kMaxLinearSpeed)
    {
      this->vx *= kMaxLinearSpeed / speed;
      this->vy *= kMaxLinearSpeed / speed;
    }
    this->wz = std::isfinite(_msg.angular().z()) ?
        std::clamp(_msg.angular().z(), -kMaxTurnRate, kMaxTurnRate) : 0.0;
    this->lastCommand = std::chrono::steady_clock::now();
  }

  private: gz::transport::Node node;
  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Link base{gz::sim::kNullEntity};
  private: gz::common::Image heightmap;
  private: std::mutex mutex;
  private: bool ready{false};
  private: double terrainSize{2000.0};
  private: double heightRange{1.0};
  private: double zOffset{0.0};
  private: double minNormalZ{std::cos(kPi / 6.0)};
  private: double vx{0.0}, vy{0.0}, wz{0.0};
  private: std::chrono::steady_clock::time_point lastCommand{};

  private: static constexpr double kPi{3.14159265358979323846};
  private: static constexpr double kMaxLinearSpeed{100.0};
  private: static constexpr double kMaxTurnRate{0.6};
  private: static constexpr double kWheelRadius{0.15};
  private: static constexpr double kWheelCenterDepth{0.17};
  private: static constexpr std::array<std::array<double, 2>, 4> kWheelOffsets{{
      {{0.38, 0.38}}, {{0.38, -0.38}},
      {{-0.38, 0.38}}, {{-0.38, -0.38}}
  }};
};
}

GZ_ADD_PLUGIN(s1::TerrainDrive, gz::sim::System,
              s1::TerrainDrive::ISystemConfigure,
              s1::TerrainDrive::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(s1::TerrainDrive, "s1::TerrainDrive")
