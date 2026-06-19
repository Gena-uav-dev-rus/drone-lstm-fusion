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
        auto entities = ecm.EntitiesByComponents(gz::sim::components::Name(model_name_));
        if (entities.empty()) return;
        const_cast<GroundTruthPlugin*>(this)->model_entity_ = entities[0];
    }

    auto poseComp = ecm.Component<gz::sim::components::Pose>(model_entity_);
    if (!poseComp) return;

    const auto &pose = poseComp->Data();
    gz::math::Vector3d position = pose.Pos();
    gz::math::Quaterniond orientation = pose.Rot();

    gz::msgs::Odometry msg;
    auto stamp = msg.mutable_header()->mutable_stamp();
    auto simTimeSec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
    auto simTimeNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count() % 1000000000;
    stamp->set_sec(simTimeSec);
    stamp->set_nsec(simTimeNsec);

    msg.mutable_pose()->mutable_position()->set_x(position.X());
    msg.mutable_pose()->mutable_position()->set_y(position.Y());
    msg.mutable_pose()->mutable_position()->set_z(position.Z());
    msg.mutable_pose()->mutable_orientation()->set_w(orientation.W());
    msg.mutable_pose()->mutable_orientation()->set_x(orientation.X());
    msg.mutable_pose()->mutable_orientation()->set_y(orientation.Y());
    msg.mutable_pose()->mutable_orientation()->set_z(orientation.Z());

    // Вычисляем скорость через численное дифференцирование позиции,
    // так как Gazebo ECM не заполняет LinearVelocity/AngularVelocity
    // компоненты автоматически без отдельной системы.
    if (has_prev_) {
        double dt = std::chrono::duration<double>(info.simTime - prev_sim_time_).count();
        if (dt > 1e-6) {
            gz::math::Vector3d lin_vel = (position - prev_position_) / dt;

            msg.mutable_twist()->mutable_linear()->set_x(lin_vel.X());
            msg.mutable_twist()->mutable_linear()->set_y(lin_vel.Y());
            msg.mutable_twist()->mutable_linear()->set_z(lin_vel.Z());

            // Угловая скорость через разницу quaternion (small-angle approx)
            gz::math::Quaterniond dq = orientation * prev_orientation_.Inverse();
            gz::math::Vector3d ang_vel(
                2.0 * dq.X() / dt,
                2.0 * dq.Y() / dt,
                2.0 * dq.Z() / dt);

            msg.mutable_twist()->mutable_angular()->set_x(ang_vel.X());
            msg.mutable_twist()->mutable_angular()->set_y(ang_vel.Y());
            msg.mutable_twist()->mutable_angular()->set_z(ang_vel.Z());
        }
    }

    prev_position_ = position;
    prev_orientation_ = orientation;
    prev_sim_time_ = info.simTime;
    const_cast<GroundTruthPlugin*>(this)->has_prev_ = true;

    pub_.Publish(msg);
}

GZ_ADD_PLUGIN(
    GroundTruthPlugin,
    gz::sim::System,
    GroundTruthPlugin::ISystemConfigure,
    GroundTruthPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GroundTruthPlugin, "ground_truth_plugin::GroundTruthPlugin")
