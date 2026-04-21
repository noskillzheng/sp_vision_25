#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "io/camera.hpp"

namespace sp_vision_25::bridge {
namespace {

using Clock = std::chrono::steady_clock;

std::string resolve_config(const std::string& package_share, const std::string& path) {
    if (path.empty())
        return package_share + "/configs/camera.yaml";
    if (path.front() == '/')
        return path;
    return package_share + "/" + path;
}

} // namespace

class CameraPreviewBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CameraPreviewBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        if (!has_parameter("config_file"))
            declare_parameter<std::string>("config_file", "configs/camera.yaml");
        if (!has_parameter("jpeg_quality"))
            declare_parameter<int64_t>("jpeg_quality", 80);
        if (!has_parameter("topic"))
            declare_parameter<std::string>("topic", "/camera/image/compressed");

        publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            get_parameter("topic").as_string(), rclcpp::SensorDataQoS());
    }

    ~CameraPreviewBridge() override {
        stop_.store(true, std::memory_order_relaxed);
        if (thread_.joinable())
            thread_.join();
    }

    void before_updating() override {
        jpeg_quality_ = static_cast<int>(get_parameter("jpeg_quality").as_int());

        const auto package_share =
            ament_index_cpp::get_package_share_directory("sp_vision_25");
        auto config_path =
            resolve_config(package_share, get_parameter("config_file").as_string());

        RCLCPP_INFO(get_logger(), "Starting camera preview with config %s", config_path.c_str());
        thread_ = std::thread(&CameraPreviewBridge::capture_loop, this, std::move(config_path));
    }

    void update() override {}

private:
    void capture_loop(std::string config_path) {
        try {
            io::Camera camera(config_path);
            const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

            while (!stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
                cv::Mat frame;
                Clock::time_point timestamp;
                camera.read(frame, timestamp);
                if (frame.empty())
                    continue;

                std::vector<uint8_t> buf;
                if (!cv::imencode(".jpg", frame, buf, encode_params))
                    continue;

                auto msg = sensor_msgs::msg::CompressedImage();
                msg.header.stamp = now();
                msg.header.frame_id = "camera";
                msg.format = "bgr8; jpeg compressed bgr8";
                msg.data = std::move(buf);
                publisher_->publish(std::move(msg));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Camera preview stopped: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    int jpeg_quality_ = 80;
};

} // namespace sp_vision_25::bridge

PLUGINLIB_EXPORT_CLASS(sp_vision_25::bridge::CameraPreviewBridge, rmcs_executor::Component)
