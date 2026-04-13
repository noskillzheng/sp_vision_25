#include "hikrobot.hpp"

#include <chrono>

#include "tools/logger.hpp"

namespace io
{
HikRobot::HikRobot(double exposure_ms, double gain, const std::string & vid_pid)
{
    hikcamera::ImageCapturer::CameraProfile profile;
    profile.exposure_time =
        std::chrono::duration<float, std::micro>(static_cast<float>(exposure_ms * 1e3));
    profile.gain = static_cast<float>(gain);

    const char * user_defined_name = vid_pid.empty() ? nullptr : vid_pid.c_str();
    camera_ = std::make_unique<hikcamera::ImageCapturer>(profile, user_defined_name);
}

HikRobot::~HikRobot() = default;

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    img = camera_->read();
    timestamp = std::chrono::steady_clock::now();
}

}  // namespace io
