#pragma once

#include "KeyFrame.h"
#include "Logging.h"
#include "Map.h"

namespace ORB_SLAM3
{
namespace InertialDebugLogging
{
// Pose fields describe the camera in world coordinates; velocity describes the
// IMU in world coordinates. Prefixes distinguish active and buffered GBA states.
// Detailed state sampling is debug-only; callers retain concise info summaries.
inline void LogState(const std::shared_ptr<spdlog::logger>& logger,
                     const char* event, const char* prefix, KeyFrame* keyframe,
                     unsigned long loopKF, const Sophus::SE3f& Twc,
                     const Eigen::Vector3f& velocity, const IMU::Bias& bias,
                     bool hasVelocity)
{
    const Eigen::Vector3f position = Twc.translation();
    const Eigen::Vector3f rotation = Twc.so3().log();
    Log(logger, spdlog::level::debug,
        "EVENT={} KF={} TIMESTAMP={:.6f} MAP={} LOOP_KF={} BIMU={} HAS_VELOCITY={} "
        "{}PX={} {}PY={} {}PZ={} {}RX={} {}RY={} {}RZ={} "
        "{}VX={} {}VY={} {}VZ={} {}VNORM={} "
        "{}BGX={} {}BGY={} {}BGZ={} {}BAX={} {}BAY={} {}BAZ={}",
        event, keyframe->mnId, keyframe->mTimeStamp, keyframe->GetMap()->GetId(),
        loopKF, keyframe->bImu, hasVelocity,
        prefix, position.x(), prefix, position.y(), prefix, position.z(),
        prefix, rotation.x(), prefix, rotation.y(), prefix, rotation.z(),
        prefix, velocity.x(), prefix, velocity.y(), prefix, velocity.z(), prefix, velocity.norm(),
        prefix, bias.bwx, prefix, bias.bwy, prefix, bias.bwz,
        prefix, bias.bax, prefix, bias.bay, prefix, bias.baz);
}

inline void LogKeyFrameState(const std::shared_ptr<spdlog::logger>& logger,
                             const char* event, KeyFrame* keyframe,
                             unsigned long loopKF = 0, const char* prefix = "")
{
    if(!logger->should_log(spdlog::level::debug) || !keyframe || keyframe->isBad())
        return;

    // Individual getters are synchronized, but the group is not an atomic
    // snapshot while LocalMapping is active (in particular at L0 and G1).
    const Sophus::SE3f Twc = keyframe->GetPoseInverse();
    const bool hasVelocity = keyframe->isVelocitySet();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    if(hasVelocity)
        velocity = keyframe->GetVelocity();
    const IMU::Bias bias = keyframe->GetImuBias();
    LogState(logger, event, prefix, keyframe, loopKF, Twc, velocity, bias, hasVelocity);
}

inline void LogGBAState(const std::shared_ptr<spdlog::logger>& logger,
                        const char* event, KeyFrame* keyframe,
                        unsigned long loopKF, bool inertialResult)
{
    if(!logger->should_log(spdlog::level::debug) || !keyframe || keyframe->isBad())
        return;

    // Unselected/new KFs may still contain scratch values from an earlier BA.
    if(keyframe->mnBAGlobalForKF != loopKF)
    {
        Log(logger, spdlog::level::debug, "EVENT={} KF={} LOOP_KF={} GBA_VALID=0",
            event, keyframe->mnId, loopKF);
        return;
    }

    const Sophus::SE3f Twc = keyframe->mTcwGBA.inverse();
    const bool hasVelocity = inertialResult && keyframe->bImu;
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    IMU::Bias bias;
    if(hasVelocity)
    {
        velocity = keyframe->mVwbGBA;
        bias = keyframe->mBiasGBA;
    }
    LogState(logger, event, "GBA_", keyframe, loopKF, Twc, velocity, bias, hasVelocity);
}
} // namespace InertialDebugLogging
} // namespace ORB_SLAM3
