#include "GroundTruthPlugin.hh"

#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/plugin/Register.hh>

using namespace ground_truth_plugin;

void GroundTruthPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager & /*eventMgr*/)
{
    model_name_ = "x500_mono_cam_0";
    if (sdf->HasElement("model_name")) {
        model_name_ = sdf->Get<std::string>("model_name");
    }

    // entity здесь — это сущность world, ищем модель по имени глобально
    auto entities = ecm.EntitiesByComponents(gz::sim::components::Name(model_name_));
    if (!entities.empty()) {
        model_entity_ = entities[0];
    }

    std::string topic = "/ground_truth/" + model_name_ + "/odometry";
    pub_ = node_.Advertise<gz::msgs::Odometry>(topic);

    gzmsg << "[GroundTruthPlugin] watching model '" << model_name_
          << "', publishing on '" << topic << "'" << std::endl;
}

void GroundTruthPlugin::PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm)
{
    if (info.paused) return;

    if (model_entity_ == gz::sim::kNullEntity) {
        // Модель могла появиться позже Configure() — пробуем найти снова
        auto entities = ecm.EntitiesByComponents(gz::sim::components::Name(model_name_));
        if (entities.empty()) return;
        const_cast<GroundTruthPlugin*>(this)->model_entity_ = entities[0];
    }

    auto poseComp = ecm.Component<gz::sim::components::Pose>(model_entity_);
    auto linVelComp = ecm.Component<gz::sim::components::LinearVelocity>(model_entity_);
    auto angVelComp = ecm.Component<gz::sim::components::AngularVelocity>(model_entity_);

    if (!poseComp) return;

    gz::msgs::Odometry msg;
    auto stamp = msg.mutable_header()->mutable_stamp();
    auto simTimeSec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
    auto simTimeNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count() % 1000000000;
    stamp->set_sec(simTimeSec);
    stamp->set_nsec(simTimeNsec);

    const auto &pose = poseComp->Data();
    msg.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
    msg.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
    msg.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
    msg.mutable_pose()->mutable_orientation()->set_w(pose.Rot().W());
    msg.mutable_pose()->mutable_orientation()->set_x(pose.Rot().X());
    msg.mutable_pose()->mutable_orientation()->set_y(pose.Rot().Y());
    msg.mutable_pose()->mutable_orientation()->set_z(pose.Rot().Z());

    if (linVelComp) {
        msg.mutable_twist()->mutable_linear()->set_x(linVelComp->Data().X());
        msg.mutable_twist()->mutable_linear()->set_y(linVelComp->Data().Y());
        msg.mutable_twist()->mutable_linear()->set_z(linVelComp->Data().Z());
    }
    if (angVelComp) {
        msg.mutable_twist()->mutable_angular()->set_x(angVelComp->Data().X());
        msg.mutable_twist()->mutable_angular()->set_y(angVelComp->Data().Y());
        msg.mutable_twist()->mutable_angular()->set_z(angVelComp->Data().Z());
    }

    pub_.Publish(msg);
}

GZ_ADD_PLUGIN(
    GroundTruthPlugin,
    gz::sim::System,
    GroundTruthPlugin::ISystemConfigure,
    GroundTruthPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GroundTruthPlugin, "ground_truth_plugin::GroundTruthPlugin")
