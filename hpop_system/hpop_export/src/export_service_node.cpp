/**
 * @file export_service_node.cpp
 * @brief ROS2 service node for ephemeris export
 */

#include <rclcpp/rclcpp.hpp>
#include <hpop_msgs/msg/satellite_state.hpp>
#include <hpop_msgs/srv/export_ephemeris.hpp>
#include <hpop_export/csv_exporter.hpp>
#include <hpop_export/oem_exporter.hpp>
#include <deque>
#include <mutex>
#include <map>
#include <filesystem>

namespace hpop_export
{

class ExportServiceNode : public rclcpp::Node
{
public:
    ExportServiceNode()
        : Node("export_service_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("output_directory", "/tmp/hpop_export");
        this->declare_parameter<int>("max_history_size", 10000);
        this->declare_parameter<std::string>("originator", "HPOP_SYSTEM");

        output_dir_ = this->get_parameter("output_directory").as_string();
        max_history_size_ = this->get_parameter("max_history_size").as_int();
        originator_ = this->get_parameter("originator").as_string();

        // Create output directory if it doesn't exist
        std::filesystem::create_directories(output_dir_);

        // Subscribe to satellite states
        state_sub_ = this->create_subscription<hpop_msgs::msg::SatelliteState>(
            "/hpop/satellite_state", 100,
            std::bind(&ExportServiceNode::stateCallback, this, std::placeholders::_1));

        // Create export service
        export_service_ = this->create_service<hpop_msgs::srv::ExportEphemeris>(
            "/hpop/export_ephemeris",
            std::bind(&ExportServiceNode::exportCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Export Service Node started");
        RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
    }

private:
    // State history per satellite
    struct SatelliteHistory
    {
        std::string name;
        std::deque<hpop_msgs::msg::SatelliteState> states;
    };

    std::map<std::string, SatelliteHistory> satellite_histories_;
    std::mutex mutex_;
    std::string output_dir_;
    std::string originator_;
    int max_history_size_;

    rclcpp::Subscription<hpop_msgs::msg::SatelliteState>::SharedPtr state_sub_;
    rclcpp::Service<hpop_msgs::srv::ExportEphemeris>::SharedPtr export_service_;

    void stateCallback(const hpop_msgs::msg::SatelliteState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::string sat_id = msg->satellite_id;
        if (sat_id.empty())
        {
            sat_id = msg->name;
        }
        if (sat_id.empty())
        {
            sat_id = "UNKNOWN_" + std::to_string(msg->norad_id);
        }

        auto& history = satellite_histories_[sat_id];
        history.name = msg->name.empty() ? sat_id : msg->name;
        history.states.push_back(*msg);

        // Limit history size
        while (static_cast<int>(history.states.size()) > max_history_size_)
        {
            history.states.pop_front();
        }
    }

    void exportCallback(
        const std::shared_ptr<hpop_msgs::srv::ExportEphemeris::Request> request,
        std::shared_ptr<hpop_msgs::srv::ExportEphemeris::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Export request received");

        std::lock_guard<std::mutex> lock(mutex_);

        // Determine which satellites to export
        std::vector<std::string> sat_ids;
        if (request->satellite_id.empty())
        {
            // Export all satellites
            for (const auto& [id, history] : satellite_histories_)
            {
                sat_ids.push_back(id);
            }
        }
        else
        {
            // Export specific satellite
            if (satellite_histories_.find(request->satellite_id) != satellite_histories_.end())
            {
                sat_ids.push_back(request->satellite_id);
            }
            else
            {
                response->success = false;
                response->message = "Satellite not found: " + request->satellite_id;
                return;
            }
        }

        if (sat_ids.empty())
        {
            response->success = false;
            response->message = "No satellite data available";
            return;
        }

        // Export based on format
        switch (request->format)
        {
            case hpop_msgs::srv::ExportEphemeris::Request::FORMAT_CSV:
                exportCsv(sat_ids, request, response);
                break;

            case hpop_msgs::srv::ExportEphemeris::Request::FORMAT_OEM:
                exportOem(sat_ids, request, response);
                break;

            default:
                response->success = false;
                response->message = "Unsupported export format: " + std::to_string(request->format);
                return;
        }
    }

    void exportCsv(const std::vector<std::string>& sat_ids,
                   const std::shared_ptr<hpop_msgs::srv::ExportEphemeris::Request>& request,
                   std::shared_ptr<hpop_msgs::srv::ExportEphemeris::Response>& response)
    {
        CsvExporter exporter;
        CsvExportOptions options;

        // Configure options
        options.include_velocity = true;
        options.include_elements = true;
        options.date_format = "ISO8601";

        uint32_t total_records = 0;
        std::string all_data;

        for (const auto& sat_id : sat_ids)
        {
            const auto& history = satellite_histories_[sat_id];

            // Convert to export format
            std::vector<EphemerisPoint> ephemeris;
            for (const auto& state : history.states)
            {
                EphemerisPoint point;
                point.epoch_jd = msgTimeToJd(state.header.stamp);
                point.x = state.position.x;
                point.y = state.position.y;
                point.z = state.position.z;
                point.vx = state.velocity.x;
                point.vy = state.velocity.y;
                point.vz = state.velocity.z;
                point.semi_major_axis = state.elements.semi_major_axis;
                point.eccentricity = state.elements.eccentricity;
                point.inclination = state.elements.inclination;
                point.raan = state.elements.raan;
                point.arg_periapsis = state.elements.arg_periapsis;
                point.true_anomaly = state.elements.true_anomaly;
                point.satellite_id = sat_id;
                point.norad_id = state.norad_id;

                ephemeris.push_back(point);
            }

            total_records += ephemeris.size();

            if (!request->output_path.empty())
            {
                // Write to file
                std::string filepath = request->output_path;
                if (sat_ids.size() > 1)
                {
                    // Multiple satellites - append sat_id
                    size_t dot_pos = filepath.rfind('.');
                    if (dot_pos != std::string::npos)
                    {
                        filepath = filepath.substr(0, dot_pos) + "_" + sat_id + filepath.substr(dot_pos);
                    }
                    else
                    {
                        filepath = filepath + "_" + sat_id + ".csv";
                    }
                }

                if (exporter.exportToFile(filepath, ephemeris, options))
                {
                    response->file_path = filepath;
                    RCLCPP_INFO(this->get_logger(), "Exported %zu records to %s",
                               ephemeris.size(), filepath.c_str());
                }
                else
                {
                    response->success = false;
                    response->message = exporter.getLastError();
                    return;
                }
            }
            else
            {
                // Return as string
                all_data += "# Satellite: " + sat_id + "\n";
                all_data += exporter.exportToString(ephemeris, options);
                all_data += "\n";
            }
        }

        response->success = true;
        response->message = "Exported " + std::to_string(total_records) + " records";
        response->num_records = total_records;
        response->ephemeris_data = all_data;
    }

    void exportOem(const std::vector<std::string>& sat_ids,
                   const std::shared_ptr<hpop_msgs::srv::ExportEphemeris::Request>& request,
                   std::shared_ptr<hpop_msgs::srv::ExportEphemeris::Response>& response)
    {
        uint32_t total_records = 0;
        std::string all_data;

        for (const auto& sat_id : sat_ids)
        {
            OemExporter exporter;
            OemExportOptions options;

            const auto& history = satellite_histories_[sat_id];

            // Configure options
            options.originator = originator_;
            options.object_name = history.name;
            options.object_id = sat_id;
            options.ref_frame = request->frame_id.empty() ? "EME2000" : request->frame_id;
            options.include_covariance = request->include_covariance;
            options.include_acceleration = request->include_acceleration;

            // Convert to OEM format (position/velocity in km, km/s)
            std::vector<OemState> states;
            for (const auto& state : history.states)
            {
                OemState oem_state;
                oem_state.epoch_jd = msgTimeToJd(state.header.stamp);
                oem_state.x = state.position.x / 1000.0;   // m -> km
                oem_state.y = state.position.y / 1000.0;
                oem_state.z = state.position.z / 1000.0;
                oem_state.vx = state.velocity.x / 1000.0;  // m/s -> km/s
                oem_state.vy = state.velocity.y / 1000.0;
                oem_state.vz = state.velocity.z / 1000.0;
                oem_state.ax = 0.0;
                oem_state.ay = 0.0;
                oem_state.az = 0.0;

                states.push_back(oem_state);
            }

            total_records += states.size();

            if (!request->output_path.empty())
            {
                // Write to file
                std::string filepath = request->output_path;
                if (sat_ids.size() > 1)
                {
                    // Multiple satellites - append sat_id
                    size_t dot_pos = filepath.rfind('.');
                    if (dot_pos != std::string::npos)
                    {
                        filepath = filepath.substr(0, dot_pos) + "_" + sat_id + filepath.substr(dot_pos);
                    }
                    else
                    {
                        filepath = filepath + "_" + sat_id + ".oem";
                    }
                }

                if (exporter.exportToFile(filepath, states, options))
                {
                    response->file_path = filepath;
                    RCLCPP_INFO(this->get_logger(), "Exported %zu records to %s (OEM)",
                               states.size(), filepath.c_str());
                }
                else
                {
                    response->success = false;
                    response->message = exporter.getLastError();
                    return;
                }
            }
            else
            {
                // Return as string
                all_data += exporter.exportToString(states, options);
                all_data += "\n";
            }
        }

        response->success = true;
        response->message = "Exported " + std::to_string(total_records) + " OEM records";
        response->num_records = total_records;
        response->ephemeris_data = all_data;
    }

    /**
     * @brief Convert ROS2 Time to Julian Date
     */
    double msgTimeToJd(const builtin_interfaces::msg::Time& stamp)
    {
        // Unix epoch (1970-01-01) in Julian Date
        const double JD_UNIX_EPOCH = 2440587.5;

        double unix_seconds = static_cast<double>(stamp.sec) +
                             static_cast<double>(stamp.nanosec) * 1e-9;

        return JD_UNIX_EPOCH + unix_seconds / 86400.0;
    }
};

} // namespace hpop_export

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hpop_export::ExportServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
