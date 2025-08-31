#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo_leo_gravity/ggm_model.hpp"
#include "gazebo_leo_gravity/legendre.hpp"

#include <string>
#include <vector>
#include <memory>

namespace gazebo_leo_gravity
{

class LeoGravityWorldPlugin : public gazebo::WorldPlugin
{
public:
    LeoGravityWorldPlugin() : update_counter_(0) {}
    virtual ~LeoGravityWorldPlugin() {}

void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
{
    world_ = _world;

    world_->SetGravity(ignition::math::Vector3d(0,0,0));

    int nmax = 20;
    if (_sdf->HasElement("nmax"))
        nmax = _sdf->Get<int>("nmax");

    std::string ggm_file = "/home/seongmin/ros2_ws/install/rsds/share/rsds/data/GGM05C.gfc";
    if (_sdf->HasElement("ggm_file"))
        ggm_file = _sdf->Get<std::string>("ggm_file");

    if (!ggm_model_.load(ggm_file, nmax))
    {
        gzerr << "[LeoGravityWorldPlugin] Failed to load GGM05C file.\n";
        return;
    }

    for (auto model : world_->Models())
    {
        if (!model->IsStatic())
            dynamic_models_.push_back(model);
    }

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&LeoGravityWorldPlugin::OnUpdate, this));
}

    void OnUpdate()
    {

        update_counter_++;
        if (update_counter_ % 10 != 0) return;

        for (auto model : dynamic_models_)
        {
            if (!model || !model->GetLink()) continue;

            ignition::math::Vector3d pos = model->WorldPose().Pos();
            ignition::math::Vector3d grav_acc = ggm_model_.acceleration(pos);

            double mass = model->GetLink()->GetInertial()->Mass();
            ignition::math::Vector3d force = grav_acc * mass;

            model->GetLink()->AddForce(force);
        }
    }

private:
    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    GGMModel ggm_model_;
    std::vector<gazebo::physics::ModelPtr> dynamic_models_;
    int update_counter_;
};

GZ_REGISTER_WORLD_PLUGIN(LeoGravityWorldPlugin)

} // namespace gazebo_leo_gravity

