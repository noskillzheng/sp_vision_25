#include <chrono>

#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/hard_sync_snapshot.hpp>

namespace sp_vision_25::bridge {

using rmcs_description::OdomImu;
using rmcs_description::PitchLink;
using rmcs_description::Tf;

class HardSyncSnapshotProvider : public rmcs_executor::Component {
public:
    HardSyncSnapshotProvider() {
        register_input("/tf", tf_);
        register_input("/predefined/timestamp", timestamp_);
        register_output("/gimbal/hard_sync_snapshot", snapshot_);
    }

    void update() override {
        Eigen::Vector3d x = *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d y = *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitY()}, *tf_);
        Eigen::Vector3d z = *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);
        Eigen::Matrix3d R;
        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;
        Eigen::Quaterniond q{R};
        q.normalize();

        snapshot_->valid = true;
        snapshot_->exposure_timestamp = *timestamp_;
        snapshot_->qw = q.w();
        snapshot_->qx = q.x();
        snapshot_->qy = q.y();
        snapshot_->qz = q.z();
    }

private:
    InputInterface<Tf> tf_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    OutputInterface<rmcs_msgs::HardSyncSnapshot> snapshot_;
};

} // namespace sp_vision_25::bridge

PLUGINLIB_EXPORT_CLASS(sp_vision_25::bridge::HardSyncSnapshotProvider, rmcs_executor::Component)
