/**
 * @file tle_service_node.cpp
 * @brief ROS2 node for TLE fetching and management
 *
 * Provides services for fetching TLEs from Celestrak and managing TLE data
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "hpop_msgs/msg/tle_data.hpp"
#include "hpop_msgs/srv/fetch_tle.hpp"

#include "hpop_tle/tle_parser.hpp"
#include "hpop_tle/tle_fetcher.hpp"

using namespace std::chrono_literals;

class TLEServiceNode : public rclcpp::Node
{
public:
    TLEServiceNode()
        : Node("tle_service_node")
    {
        // Declare parameters
        this->declare_parameter("cache_duration_hours", 24);
        this->declare_parameter("auto_refresh", false);
        this->declare_parameter("refresh_interval_hours", 6);

        cache_duration_ = std::chrono::hours(
            this->get_parameter("cache_duration_hours").as_int());

        // Create service
        fetch_service_ = this->create_service<hpop_msgs::srv::FetchTLE>(
            "/hpop/fetch_tle",
            std::bind(&TLEServiceNode::handleFetchTLE, this,
                      std::placeholders::_1, std::placeholders::_2));

        // TLE publisher (for broadcasting fetched TLEs)
        tle_publisher_ = this->create_publisher<hpop_msgs::msg::TLEData>(
            "/hpop/tle_updates", 10);

        // Status publisher
        status_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/hpop/tle_status", 10);

        // Auto refresh timer if enabled
        if (this->get_parameter("auto_refresh").as_bool())
        {
            int refresh_hours = this->get_parameter("refresh_interval_hours").as_int();
            refresh_timer_ = this->create_wall_timer(
                std::chrono::hours(refresh_hours),
                std::bind(&TLEServiceNode::refreshAllTLEs, this));
        }

        RCLCPP_INFO(this->get_logger(), "TLE Service Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Cache duration: %ld hours",
                    cache_duration_.count());
    }

private:
    rclcpp::Service<hpop_msgs::srv::FetchTLE>::SharedPtr fetch_service_;
    rclcpp::Publisher<hpop_msgs::msg::TLEData>::SharedPtr tle_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr refresh_timer_;

    hpop_tle::TLEFetcher fetcher_;
    hpop_tle::TLECache cache_;
    std::chrono::hours cache_duration_;

    // Track which NORAD IDs are being monitored
    std::vector<uint32_t> monitored_ids_;

    void handleFetchTLE(
        const std::shared_ptr<hpop_msgs::srv::FetchTLE::Request> request,
        std::shared_ptr<hpop_msgs::srv::FetchTLE::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received FetchTLE request for %zu satellites",
                    request->norad_ids.size());

        response->success = true;
        std::vector<hpop_msgs::msg::TLEData> fetched_tles;

        for (uint32_t norad_id : request->norad_ids)
        {
            // Check cache first
            auto cached = cache_.get(norad_id, cache_duration_);
            if (cached.has_value())
            {
                RCLCPP_DEBUG(this->get_logger(), "Using cached TLE for NORAD %u", norad_id);
                fetched_tles.push_back(convertToMsg(cached.value()));
                continue;
            }

            // Fetch from Celestrak
            RCLCPP_INFO(this->get_logger(), "Fetching TLE for NORAD %u from Celestrak", norad_id);
            auto result = fetcher_.fetchByNoradId(norad_id);

            if (!result.success || result.tles.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to fetch TLE for NORAD %u: %s",
                            norad_id, result.error_message.c_str());
                response->success = false;
                response->message += "Failed for NORAD " + std::to_string(norad_id) +
                                     ": " + result.error_message + "; ";
                continue;
            }

            // Cache and convert
            const auto& tle = result.tles[0];
            cache_.put(tle);
            fetched_tles.push_back(convertToMsg(tle));

            // Add to monitored list
            if (std::find(monitored_ids_.begin(), monitored_ids_.end(), norad_id) ==
                monitored_ids_.end())
            {
                monitored_ids_.push_back(norad_id);
            }

            RCLCPP_INFO(this->get_logger(), "Successfully fetched TLE for %s (NORAD %u)",
                        tle.satellite_name.c_str(), norad_id);
        }

        response->tle_data = fetched_tles;

        if (response->success)
        {
            response->message = "Successfully fetched " +
                                std::to_string(fetched_tles.size()) + " TLEs";
        }

        // Publish status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = response->message;
        status_publisher_->publish(status_msg);
    }

    hpop_msgs::msg::TLEData convertToMsg(const hpop_tle::TLEData& tle)
    {
        hpop_msgs::msg::TLEData msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "eci";

        msg.satellite_name = tle.satellite_name;
        msg.norad_id = tle.norad_id;
        msg.classification = std::string(1, tle.classification);
        msg.intl_designator = tle.intl_designator;

        msg.epoch_year = tle.epoch_year;
        msg.epoch_day = tle.epoch_day;
        // Convert Julian Date to ROS time (epoch_time)
        // JD to Unix time: (JD - 2440587.5) * 86400
        double unix_time = (tle.epochJD() - 2440587.5) * 86400.0;
        msg.epoch_time.sec = static_cast<int32_t>(unix_time);
        msg.epoch_time.nanosec = static_cast<uint32_t>((unix_time - msg.epoch_time.sec) * 1e9);

        msg.mean_motion_dot = tle.mean_motion_dot;
        msg.mean_motion_ddot = tle.mean_motion_ddot;
        msg.bstar = tle.bstar;

        msg.inclination = tle.inclination;
        msg.raan = tle.raan;
        msg.eccentricity = tle.eccentricity;
        msg.arg_periapsis = tle.arg_periapsis;
        msg.mean_anomaly = tle.mean_anomaly;
        msg.mean_motion = tle.mean_motion;
        msg.revolution_number = tle.revolution_number;

        msg.line1 = tle.line1;
        msg.line2 = tle.line2;

        return msg;
    }

    void refreshAllTLEs()
    {
        if (monitored_ids_.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "No monitored TLEs to refresh");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Auto-refreshing %zu TLEs",
                    monitored_ids_.size());

        auto result = fetcher_.fetchByNoradIds(monitored_ids_);

        if (result.success)
        {
            for (const auto& tle : result.tles)
            {
                cache_.put(tle);
                auto msg = convertToMsg(tle);
                tle_publisher_->publish(msg);
            }
            RCLCPP_INFO(this->get_logger(), "Refreshed %zu TLEs", result.tles.size());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "TLE refresh failed: %s",
                        result.error_message.c_str());
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TLEServiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
