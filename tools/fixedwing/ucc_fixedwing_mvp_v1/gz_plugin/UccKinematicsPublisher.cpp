#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <gz/common/Console.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace ucc
{
namespace sim
{
class KinematicsPublisher final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm)) {
      throw std::runtime_error(
        "UccKinematicsPublisher must be attached to a Gazebo model");
    }

    const auto linkResult = _sdf->Get<std::string>("link_name", "base_link");
    const auto topicResult = _sdf->Get<std::string>(
      "topic", "/ucc/fixed_wing/kinematics");
    const auto rateResult = _sdf->Get<double>("publish_rate_hz", 250.0);

    this->linkName = linkResult.first;
    this->topic = topicResult.first;
    const double publishRateHz = rateResult.first;
    if (publishRateHz <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }

    const gz::sim::Entity linkEntity = this->model.LinkByName(_ecm, this->linkName);
    if (linkEntity == gz::sim::kNullEntity) {
      throw std::runtime_error(
        "UccKinematicsPublisher link not found: " + this->linkName);
    }

    this->link = gz::sim::Link(linkEntity);
    this->link.EnableVelocityChecks(_ecm, true);
    this->link.EnableAccelerationChecks(_ecm, true);

    this->publishPeriod = std::chrono::nanoseconds(
      static_cast<std::int64_t>(1.0e9 / publishRateHz));
    this->publisher = this->node.Advertise<gz::msgs::Double_V>(this->topic);
    if (!this->publisher.Valid()) {
      throw std::runtime_error(
        "UccKinematicsPublisher failed to advertise: " + this->topic);
    }

    gzmsg << "[UCC_KINEMATICS] READY model=" << this->model.Name(_ecm)
          << " link=" << this->linkName
          << " topic=" << this->topic
          << " rate_hz=" << publishRateHz << std::endl;
  }

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused) {
      return;
    }

    if (_info.simTime < this->lastSimTime) {
      this->nextPublishTime = _info.simTime;
    }
    this->lastSimTime = _info.simTime;

    if (_info.simTime < this->nextPublishTime) {
      return;
    }

    const auto pose = this->link.WorldPose(_ecm);
    const auto linearVelocity = this->link.WorldLinearVelocity(_ecm);
    const auto angularVelocityWorld = this->link.WorldAngularVelocity(_ecm);
    const auto linearAcceleration = this->link.WorldLinearAcceleration(_ecm);
    const auto angularAccelerationWorld = this->link.WorldAngularAcceleration(_ecm);

    if (!pose || !linearVelocity || !angularVelocityWorld ||
        !linearAcceleration || !angularAccelerationWorld) {
      if (!this->reportedMissingState) {
        gzwarn << "[UCC_KINEMATICS] waiting for complete link state" << std::endl;
        this->reportedMissingState = true;
      }
      return;
    }
    this->reportedMissingState = false;

    const auto angularVelocityBody =
      pose->Rot().RotateVectorReverse(*angularVelocityWorld);
    const auto angularAccelerationBody =
      pose->Rot().RotateVectorReverse(*angularAccelerationWorld);
    const auto sourceTimeNs =
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();

    gz::msgs::Double_V message;
    message.mutable_data()->Reserve(21);
    message.add_data(1.0);  // ucc.fixedwing.kinematics.v1
    message.add_data(static_cast<double>(sourceTimeNs));
    message.add_data(pose->Pos().X());
    message.add_data(pose->Pos().Y());
    message.add_data(pose->Pos().Z());
    message.add_data(pose->Rot().X());
    message.add_data(pose->Rot().Y());
    message.add_data(pose->Rot().Z());
    message.add_data(pose->Rot().W());
    message.add_data(linearVelocity->X());
    message.add_data(linearVelocity->Y());
    message.add_data(linearVelocity->Z());
    message.add_data(angularVelocityBody.X());
    message.add_data(angularVelocityBody.Y());
    message.add_data(angularVelocityBody.Z());
    message.add_data(linearAcceleration->X());
    message.add_data(linearAcceleration->Y());
    message.add_data(linearAcceleration->Z());
    message.add_data(angularAccelerationBody.X());
    message.add_data(angularAccelerationBody.Y());
    message.add_data(angularAccelerationBody.Z());

    if (!this->publisher.Publish(message)) {
      gzwarn << "[UCC_KINEMATICS] publish failed on " << this->topic << std::endl;
    }

    do {
      this->nextPublishTime += this->publishPeriod;
    } while (this->nextPublishTime <= _info.simTime);
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Link link{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::string linkName{"base_link"};
  std::string topic{"/ucc/fixed_wing/kinematics"};
  std::chrono::steady_clock::duration publishPeriod{std::chrono::milliseconds(4)};
  std::chrono::steady_clock::duration nextPublishTime{std::chrono::nanoseconds(0)};
  std::chrono::steady_clock::duration lastSimTime{std::chrono::nanoseconds(0)};
  bool reportedMissingState{false};
};
}  // namespace sim
}  // namespace ucc

GZ_ADD_PLUGIN(
  ucc::sim::KinematicsPublisher,
  gz::sim::System,
  ucc::sim::KinematicsPublisher::ISystemConfigure,
  ucc::sim::KinematicsPublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  ucc::sim::KinematicsPublisher,
  "ucc::sim::systems::KinematicsPublisher")
