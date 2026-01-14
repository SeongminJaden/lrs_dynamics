#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo_leo_gravity/ggm_model.hpp"

#include <string>
#include <vector>
#include <memory>
#include <unordered_set>
#include <omp.h>

namespace gazebo_leo_gravity
{

class LeoGravityWorldPlugin : public gazebo::WorldPlugin
{
public:
    LeoGravityWorldPlugin() = default;
    ~LeoGravityWorldPlugin() override = default;

    void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
    {
        world_ = world;

        // Disable default gravity
        world_->SetGravity(ignition::math::Vector3d::Zero);

        // Read configuration from SDF
        int nmax = 20;
        if (sdf->HasElement("nmax"))
            nmax = sdf->Get<int>("nmax");

        if (sdf->HasElement("update_rate"))
            update_skip_ = static_cast<int>(1000.0 / sdf->Get<double>("update_rate"));

        // OpenMP thread count
        int num_threads = omp_get_max_threads();
        if (sdf->HasElement("threads"))
            num_threads = sdf->Get<int>("threads");
        omp_set_num_threads(num_threads);

        std::string ggm_file = "/home/seongmin/ros2_ws/install/rsds/share/rsds/data/GGM05C.gfc";
        if (sdf->HasElement("ggm_file"))
            ggm_file = sdf->Get<std::string>("ggm_file");

        // Create per-thread GGM models for thread safety
        ggm_models_.resize(num_threads);
        for (int i = 0; i < num_threads; ++i)
        {
            if (!ggm_models_[i].load(ggm_file, nmax))
            {
                gzerr << "[LeoGravityWorldPlugin] Failed to load GGM file: " << ggm_file << "\n";
                return;
            }
        }

        gzmsg << "[LeoGravityWorldPlugin] Initialized: nmax=" << nmax
              << ", threads=" << num_threads
              << ", update_skip=" << update_skip_ << "\n";

        // Initial model scan
        RefreshDynamicModels();

        // Register callbacks
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&LeoGravityWorldPlugin::OnUpdate, this));

        add_connection_ = gazebo::event::Events::ConnectAddEntity(
            std::bind(&LeoGravityWorldPlugin::OnModelAdded, this, std::placeholders::_1));
    }

private:
    void OnUpdate()
    {
        ++update_counter_;
        if (update_counter_ % update_skip_ != 0) return;

        // Periodic refresh
        if (update_counter_ % (update_skip_ * 100) == 0)
            RefreshDynamicModels();

        const int n_models = static_cast<int>(dynamic_models_.size());
        if (n_models == 0) return;

        // OpenMP parallel loop over models
        #pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < n_models; ++i)
        {
            auto& model = dynamic_models_[i];
            if (!model) continue;

            auto link = model->GetLink();
            if (!link) continue;

            const int tid = omp_get_thread_num();
            const ignition::math::Vector3d pos = model->WorldPose().Pos();
            const ignition::math::Vector3d grav_acc = ggm_models_[tid].acceleration(pos);

            const double mass = link->GetInertial()->Mass();
            link->AddForce(grav_acc * mass);
        }
    }

    void OnModelAdded(const std::string& entity_name)
    {
        auto model = world_->ModelByName(entity_name);
        if (model && !model->IsStatic())
        {
            if (tracked_models_.find(entity_name) == tracked_models_.end())
            {
                dynamic_models_.push_back(model);
                tracked_models_.insert(entity_name);
                gzmsg << "[LeoGravityWorldPlugin] Tracking: " << entity_name << "\n";
            }
        }
    }

    void RefreshDynamicModels()
    {
        dynamic_models_.clear();
        tracked_models_.clear();

        for (auto& model : world_->Models())
        {
            if (model && !model->IsStatic())
            {
                dynamic_models_.push_back(model);
                tracked_models_.insert(model->GetName());
            }
        }
    }

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::event::ConnectionPtr add_connection_;

    std::vector<GGMModel> ggm_models_;  // Per-thread models
    std::vector<gazebo::physics::ModelPtr> dynamic_models_;
    std::unordered_set<std::string> tracked_models_;

    int update_counter_ = 0;
    int update_skip_ = 10;
};

GZ_REGISTER_WORLD_PLUGIN(LeoGravityWorldPlugin)

} // namespace gazebo_leo_gravity
