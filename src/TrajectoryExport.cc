#include "TrajectoryExport.h"
#include "Atlas.h"
#include "KeyFrame.h"
#include "Map.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <stdexcept>

namespace ORB_SLAM3
{
namespace trajectory_export
{
namespace
{
bool valid_pose(const Sophus::SE3f& pose)
{
    return pose.translation().allFinite() && pose.unit_quaternion().coeffs().allFinite() &&
           std::abs(pose.unit_quaternion().squaredNorm() - 1.0f) <= 1e-4f;
}

std::vector<KeyFrame*> map_keyframes(Map* map)
{
    std::vector<KeyFrame*> keyframes;
    if (!map) return keyframes;
    for (KeyFrame* keyframe : map->GetAllKeyFrames())
        if (keyframe && !keyframe->isBad() && keyframe->GetMap() == map)
            keyframes.push_back(keyframe);
    std::sort(keyframes.begin(), keyframes.end(), KeyFrame::lId);
    return keyframes;
}

struct PreparedMap
{
    Map* map;
    std::vector<KeyFrame*> keyframes;
    bool inertial;
    Sophus::SE3f origin_in_map;
    Sophus::SE3f body_from_camera;
};

bool valid_calibration(const IMU::Calib& calibration)
{
    return calibration.mbIsSet && valid_pose(calibration.mTbc) && valid_pose(calibration.mTcb) &&
           (calibration.mTbc * calibration.mTcb).matrix().isApprox(Eigen::Matrix4f::Identity(), 1e-5f);
}

PreparedMap prepare_map(Map* map, bool inertial)
{
    PreparedMap prepared{map, map_keyframes(map), inertial, {}, {}};
    if (prepared.keyframes.empty())
        throw std::invalid_argument("Trajectory export requires a map with valid keyframes");
    KeyFrame* origin = prepared.keyframes.front();
    if (!valid_pose(origin->GetPoseInverse()) || !std::isfinite(origin->mTimeStamp) ||
        !std::isfinite(origin->mTimeStamp * 1e9))
        throw std::invalid_argument("Trajectory origin keyframe has invalid pose or timestamp");
    if (inertial)
    {
        if (!valid_calibration(origin->mImuCalib))
            throw std::invalid_argument("Trajectory origin keyframe has no valid IMU calibration");
        prepared.body_from_camera = origin->mImuCalib.mTbc;
        prepared.origin_in_map = origin->GetImuPose();
    }
    else
        prepared.origin_in_map = origin->GetPoseInverse();
    if (!valid_pose(prepared.origin_in_map))
        throw std::invalid_argument("Trajectory origin transform is invalid");
    return prepared;
}

bool matching_calibration(KeyFrame* keyframe, const PreparedMap& prepared)
{
    if (!prepared.inertial) return true;
    const IMU::Calib& calibration = keyframe->mImuCalib;
    return valid_calibration(calibration) &&
           (calibration.mTbc.translation() - prepared.body_from_camera.translation()).norm() <= 1e-5f &&
           calibration.mTbc.rotationMatrix().isApprox(prepared.body_from_camera.rotationMatrix(), 1e-5f);
}

void check_history(const FrameHistory& history)
{
    const std::size_t count = history.relative_poses.size();
    if (history.references.size() != count || history.timestamps_sec.size() != count ||
        history.lost.size() != count)
        throw std::invalid_argument("Trajectory history lists have different lengths");
}

void copy_transform(const Sophus::SE3f& pose, TrajectoryTransform& result)
{
    for (int index = 0; index < 3; ++index)
        result.translation[index] = pose.translation()[index];
    for (int index = 0; index < 4; ++index)
        result.quaternion_xyzw[index] = pose.unit_quaternion().coeffs()[index];
}

void write_pose(std::ostream& output, double timestamp_sec, const Sophus::SE3f& pose,
                TrajectoryFileSummary& summary)
{
    const Eigen::Vector3f& translation = pose.translation();
    const Eigen::Quaternionf& rotation = pose.unit_quaternion();
    // Preserve EuRoC's ns convention. The source is still a rounded double in
    // seconds; consumers must not claim exact original integer sensor timestamps.
    output << std::setprecision(6) << timestamp_sec * 1e9 << ' ' << std::setprecision(9)
           << translation.x() << ' ' << translation.y() << ' ' << translation.z() << ' '
           << rotation.x() << ' ' << rotation.y() << ' ' << rotation.z() << ' ' << rotation.w() << '\n';
    if (summary.written == 0) summary.first_timestamp_sec = timestamp_sec;
    summary.last_timestamp_sec = timestamp_sec;
    ++summary.written;
}

TrajectoryFileSummary write_frames(const std::string& filename, const PreparedMap& prepared,
                                  const FrameHistory& history)
{
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.imbue(std::locale::classic());
    output.open(filename);
    output << std::fixed;

    TrajectoryFileSummary summary;
    auto reference = history.references.begin();
    auto timestamp = history.timestamps_sec.begin();
    auto lost = history.lost.begin();
    // Reuse storage across frames while detecting null/cyclic culled-KF chains.
    std::vector<KeyFrame*> ancestors;
    for (const Sophus::SE3f& relative : history.relative_poses)
    {
        KeyFrame* keyframe = *reference++;
        const double time = *timestamp++;
        const bool frame_lost = *lost++;
        ++summary.considered;
        if (frame_lost) { ++summary.lost; continue; }
        if (!std::isfinite(time) || !std::isfinite(time * 1e9))
        { ++summary.invalid_timestamp; continue; }
        if (!valid_pose(relative)) { ++summary.invalid_pose; continue; }

        // Retain the existing spanning-tree reconstruction for culled references.
        Sophus::SE3f reference_from_map;
        ancestors.clear();
        while (keyframe && keyframe->isBad())
        {
            if (std::find(ancestors.begin(), ancestors.end(), keyframe) != ancestors.end())
            { keyframe = nullptr; break; }
            ancestors.push_back(keyframe);
            if (!valid_pose(keyframe->mTcp)) { keyframe = nullptr; break; }
            reference_from_map = reference_from_map * keyframe->mTcp;
            keyframe = keyframe->GetParent();
        }
        if (!keyframe) { ++summary.invalid_reference; continue; }
        if (keyframe->GetMap() != prepared.map) { ++summary.other_map; continue; }
        if (!valid_pose(keyframe->GetPose()) || !matching_calibration(keyframe, prepared))
        { ++summary.invalid_pose; continue; }

        reference_from_map = reference_from_map * keyframe->GetPose() * prepared.origin_in_map;
        Sophus::SE3f sensor_from_origin = relative * reference_from_map;
        if (prepared.inertial)
            sensor_from_origin = keyframe->mImuCalib.mTbc * sensor_from_origin;
        if (!valid_pose(sensor_from_origin)) { ++summary.invalid_pose; continue; }
        write_pose(output, time, sensor_from_origin.inverse(), summary);
    }
    // Buffered writes occur only during export. close() flushes/checks the final
    // buffer; no row uses endl, unitbuf or an explicit per-row flush.
    output.close();
    return summary;
}

TrajectoryFileSummary write_keyframes(const std::string& filename, const PreparedMap& prepared)
{
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.imbue(std::locale::classic());
    output.open(filename);
    output << std::fixed;

    TrajectoryFileSummary summary;
    for (KeyFrame* keyframe : prepared.keyframes)
    {
        ++summary.considered;
        if (!std::isfinite(keyframe->mTimeStamp) || !std::isfinite(keyframe->mTimeStamp * 1e9))
        { ++summary.invalid_timestamp; continue; }
        if (!valid_pose(keyframe->GetPoseInverse()) || !matching_calibration(keyframe, prepared))
        { ++summary.invalid_pose; continue; }
        // Keep the existing KF world origin, which differs from frame-row origin.
        const Sophus::SE3f pose = prepared.inertial ? keyframe->GetImuPose() : keyframe->GetPoseInverse();
        if (!valid_pose(pose)) { ++summary.invalid_pose; continue; }
        write_pose(output, keyframe->mTimeStamp, pose, summary);
    }
    output.close();
    return summary;
}
}

Map* SelectMap(const std::vector<Map*>& maps)
{
    Map* selected = nullptr;
    std::size_t largest = 0;
    for (Map* map : maps)
    {
        const std::size_t count = map_keyframes(map).size();
        if (count > largest || (count != 0 && count == largest &&
                              selected && map->GetId() < selected->GetId()))
        { selected = map; largest = count; }
    }
    return selected;
}

TrajectoryFileSummary SaveFrames(const std::string& filename, Map* map,
                                const FrameHistory& history, bool inertial)
{
    check_history(history);
    return write_frames(filename, prepare_map(map, inertial), history);
}

TrajectoryFileSummary SaveKeyFrames(const std::string& filename, Map* map, bool inertial)
{
    return write_keyframes(filename, prepare_map(map, inertial));
}

TrajectoryExportResult SaveEvaluation(bool shutdown_completed, Atlas& atlas,
                                    const FrameHistory& history, bool inertial,
                                    const std::string& frame_filename,
                                    const std::string& keyframe_filename)
{
    // System supplies completion, not the earlier isShutDown() entry flag. Check
    // it before reading Atlas/history or touching an output path.
    if (!shutdown_completed)
        throw std::logic_error("Evaluation trajectory export requires completed Shutdown");
    if (frame_filename.empty() || keyframe_filename.empty())
        throw std::invalid_argument("Trajectory output paths must not be empty");
    std::error_code error;
    const bool same_file = std::filesystem::equivalent(frame_filename, keyframe_filename, error);
    if (same_file || std::filesystem::weakly_canonical(frame_filename) ==
                     std::filesystem::weakly_canonical(keyframe_filename))
        throw std::invalid_argument("Frame and keyframe trajectory output paths must differ");
    check_history(history);

    TrajectoryExportResult result;
    result.pose_target = inertial ? TrajectoryPoseTarget::ImuBody : TrajectoryPoseTarget::LeftCamera;
    const std::vector<Map*> maps = atlas.GetAllMaps();
    Map* map = SelectMap(maps);
    if (!map)
    {
        result.status = maps.empty() ? TrajectoryExportStatus::NoMap : TrajectoryExportStatus::NoKeyFrames;
        return result;
    }
    const PreparedMap prepared = prepare_map(map, inertial);
    result.map_selected = true;
    result.map_id = map->GetId();
    result.map_keyframes = prepared.keyframes.size();
    result.origin_keyframe_id = prepared.keyframes.front()->mnId;
    result.origin_timestamp_sec = prepared.keyframes.front()->mTimeStamp;
    result.imu_initialized = inertial && map->isImuInitialized();
    copy_transform(prepared.origin_in_map, result.frame_origin_in_map);
    if (inertial)
    {
        result.body_from_camera_valid = true;
        copy_transform(prepared.body_from_camera, result.body_from_camera);
    }
    // One prepared map/origin drives both existing EuRoC reconstruction paths.
    // Do not promise a complete pair until both streams have closed successfully.
    result.frames = write_frames(frame_filename, prepared, history);
    result.keyframes = write_keyframes(keyframe_filename, prepared);
    if (result.frames.written == 0)
        result.status = TrajectoryExportStatus::NoFramePoses;
    else if (result.keyframes.written == 0)
        result.status = TrajectoryExportStatus::NoKeyFrames;
    else
        result.status = TrajectoryExportStatus::Complete;
    return result;
}
}
}
