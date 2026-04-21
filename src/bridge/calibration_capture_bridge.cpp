#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "io/camera.hpp"

namespace sp_vision_25::bridge {
namespace {

using Clock = std::chrono::steady_clock;
using rmcs_description::OdomImu;
using rmcs_description::PitchLink;
using rmcs_description::Tf;

std::string resolve_config(const std::string& package_share, const std::string& path) {
    if (path.empty())
        return package_share + "/configs/calibration.yaml";
    if (path.front() == '/')
        return path;
    return package_share + "/" + path;
}

Eigen::Quaterniond extract_gimbal_quaternion(const Tf& tf) {
    Eigen::Vector3d x = *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, tf);
    Eigen::Vector3d y = *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitY()}, tf);
    Eigen::Vector3d z = *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitZ()}, tf);
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    return Eigen::Quaterniond{R}.normalized();
}

void save_quaternion(const std::string& path, const Eigen::Quaterniond& q) {
    std::ofstream file(path);
    Eigen::Vector4d xyzw = q.coeffs();
    file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
}

} // namespace

class CalibrationCaptureBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CalibrationCaptureBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        if (!has_parameter("config_file"))
            declare_parameter<std::string>("config_file", "configs/calibration.yaml");
        if (!has_parameter("output_folder"))
            declare_parameter<std::string>("output_folder", "/tmp/calibration_capture");
        if (!has_parameter("pattern_cols"))
            declare_parameter<int64_t>("pattern_cols", 10);
        if (!has_parameter("pattern_rows"))
            declare_parameter<int64_t>("pattern_rows", 7);
        if (!has_parameter("jpeg_quality"))
            declare_parameter<int64_t>("jpeg_quality", 80);
        if (!has_parameter("topic"))
            declare_parameter<std::string>("topic", "/camera/image/compressed");

        register_input("/tf", tf_);
        register_input("/remote/switch/left", switch_left_);

        publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            get_parameter("topic").as_string(), rclcpp::SensorDataQoS());
    }

    ~CalibrationCaptureBridge() override {
        stop_.store(true, std::memory_order_relaxed);
        if (camera_thread_.joinable())
            camera_thread_.join();
    }

    void before_updating() override {
        pattern_size_ = cv::Size(
            static_cast<int>(get_parameter("pattern_cols").as_int()),
            static_cast<int>(get_parameter("pattern_rows").as_int()));
        jpeg_quality_ = static_cast<int>(get_parameter("jpeg_quality").as_int());
        output_folder_ = get_parameter("output_folder").as_string();
        std::filesystem::create_directories(output_folder_);

        const auto package_share =
            ament_index_cpp::get_package_share_directory("sp_vision_25");
        auto config_path =
            resolve_config(package_share, get_parameter("config_file").as_string());

        RCLCPP_INFO(
            get_logger(), "Calibration capture: config=%s output=%s pattern=%dx%d",
            config_path.c_str(), output_folder_.c_str(),
            pattern_size_.width, pattern_size_.height);

        camera_thread_ = std::thread(
            &CalibrationCaptureBridge::camera_loop, this, std::move(config_path));
    }

    void update() override {
        bool switch_down = (*switch_left_ == rmcs_msgs::Switch::DOWN);
        bool edge = switch_down && !prev_switch_down_;
        prev_switch_down_ = switch_down;

        if (!edge)
            return;

        cv::Mat frame;
        bool detected = false;
        {
            std::lock_guard lock(frame_mutex_);
            if (latest_frame_.empty())
                return;
            frame = latest_frame_.clone();
            detected = board_detected_;
        }

        if (!detected) {
            RCLCPP_WARN(get_logger(), "No calibration board detected, skipping save");
            return;
        }

        save_count_++;
        auto img_path = fmt::format("{}/{}.jpg", output_folder_, save_count_);
        auto q_path = fmt::format("{}/{}.txt", output_folder_, save_count_);

        cv::imwrite(img_path, frame);
        auto q = extract_gimbal_quaternion(*tf_);
        save_quaternion(q_path, q);

        RCLCPP_INFO(get_logger(), "[%d] Saved to %s", save_count_.load(), output_folder_.c_str());
    }

private:
    void camera_loop(std::string config_path) {
        try {
            io::Camera camera(config_path);
            const std::vector<int> encode_params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

            while (!stop_.load(std::memory_order_relaxed) && rclcpp::ok()) {
                cv::Mat frame;
                Clock::time_point timestamp;
                camera.read(frame, timestamp);
                if (frame.empty())
                    continue;

                std::vector<cv::Point2f> centers;
                bool detected = cv::findCirclesGrid(frame, pattern_size_, centers);

                {
                    std::lock_guard lock(frame_mutex_);
                    latest_frame_ = frame.clone();
                    board_detected_ = detected;
                }

                cv::Mat overlay = frame.clone();
                cv::drawChessboardCorners(overlay, pattern_size_, centers, detected);
                cv::putText(
                    overlay,
                    detected ? fmt::format("DETECTED [saved: {}]", save_count_.load())
                             : fmt::format("NO BOARD [saved: {}]", save_count_.load()),
                    {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                    detected ? cv::Scalar{0, 255, 0} : cv::Scalar{0, 0, 255}, 2);

                std::vector<uint8_t> buf;
                if (!cv::imencode(".jpg", overlay, buf, encode_params))
                    continue;

                auto msg = sensor_msgs::msg::CompressedImage();
                msg.header.stamp = now();
                msg.header.frame_id = "camera";
                msg.format = "bgr8; jpeg compressed bgr8";
                msg.data = std::move(buf);
                publisher_->publish(std::move(msg));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Calibration capture stopped: %s", e.what());
        }
    }

    InputInterface<Tf> tf_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    std::thread camera_thread_;
    std::atomic<bool> stop_{false};

    std::mutex frame_mutex_;
    cv::Mat latest_frame_;
    bool board_detected_ = false;

    cv::Size pattern_size_;
    int jpeg_quality_ = 80;
    std::string output_folder_;
    std::atomic<int> save_count_{0};
    bool prev_switch_down_ = true;
};

} // namespace sp_vision_25::bridge

PLUGINLIB_EXPORT_CLASS(sp_vision_25::bridge::CalibrationCaptureBridge, rmcs_executor::Component)
