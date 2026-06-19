#ifndef GROUND_TRUTH_PLUGIN_HH
#define GROUND_TRUTH_PLUGIN_HH

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <chrono>
#include <string>

namespace ground_truth_plugin
{

class GroundTruthPlugin:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
    GroundTruthPlugin() = default;
    ~GroundTruthPlugin() override = default;

    void Configure(const gz::sim::Entity &entity,
                    const std::shared_ptr<const sdf::Element> &sdf,
                    gz::sim::EntityComponentManager &ecm,
                    gz::sim::EventManager &eventMgr) override;

    void PostUpdate(const gz::sim::UpdateInfo &info,
                     const gz::sim::EntityComponentManager &ecm) override;

private:
    std::string model_name_;
    gz::sim::Entity model_entity_{gz::sim::kNullEntity};
    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_;

    // Для численного дифференцирования позиции -> скорость
    bool has_prev_ = false;
    gz::math::Vector3d prev_position_;
    gz::math::Quaterniond prev_orientation_;
    std::chrono::steady_clock::duration prev_sim_time_{0};
};

}  // namespace ground_truth_plugin

#endif
