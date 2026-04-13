#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "hikcamera/image_capturer.hpp"
#include "io/camera.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
    HikRobot(double exposure_ms, double gain, const std::string & vid_pid);
    ~HikRobot() override;
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
    std::unique_ptr<hikcamera::ImageCapturer> camera_;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
