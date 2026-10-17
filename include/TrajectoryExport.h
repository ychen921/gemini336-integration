#ifndef ORB_SLAM3_TRAJECTORY_EXPORT_H
#define ORB_SLAM3_TRAJECTORY_EXPORT_H

#include <array>
#include <cstdint>
#include <list>
#include <string>
#include <vector>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{
class Atlas;
class KeyFrame;
class Map;

enum class TrajectoryExportStatus { Complete, NoMap, NoKeyFrames, NoFramePoses };
enum class TrajectoryPoseTarget { LeftCamera, ImuBody };

// T_a_b maps points from b to a. Metres and quaternion order x,y,z,w.
struct TrajectoryTransform
{
    std::array<double, 3> translation{};
    std::array<double, 4> quaternion_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct TrajectoryFileSummary
{
    std::uint64_t considered = 0;
    std::uint64_t written = 0;
    std::uint64_t lost = 0;
    std::uint64_t invalid_reference = 0;
    std::uint64_t other_map = 0;
    std::uint64_t invalid_pose = 0;
    std::uint64_t invalid_timestamp = 0;
    // Valid only when written != 0. Original Core double seconds, not exact ns.
    double first_timestamp_sec = 0.0;
    double last_timestamp_sec = 0.0;
};

// Own values only; no map/keyframe pointers survive export. Both text files use
// timestamp_ns tx ty tz qx qy qz qw, with Core float poses and double timestamps.
struct TrajectoryExportResult
{
    TrajectoryExportStatus status = TrajectoryExportStatus::NoMap;
    bool map_selected = false;
    std::uint64_t map_id = 0;
    // Non-bad KFs belonging to the selected map, before pose/timestamp filtering.
    std::uint64_t map_keyframes = 0;
    std::uint64_t origin_keyframe_id = 0;
    double origin_timestamp_sec = 0.0;
    TrajectoryPoseTarget pose_target = TrajectoryPoseTarget::LeftCamera;
    bool imu_initialized = false; // Meaningful only for ImuBody.
    bool body_from_camera_valid = false;
    // Read only when body_from_camera_valid: the actual reference-KF T_b_c.
    TrajectoryTransform body_from_camera;
    // Frame rows are T_origin_sensor; keyframe rows retain T_map_sensor.
    // T_map_camera(frame) = frame_origin_in_map * frame_row * T_b_c for IMU,
    // or frame_origin_in_map * frame_row for stereo. Keyframe rows need no
    // origin multiplication. Identity defaults are unavailable if !map_selected.
    TrajectoryTransform frame_origin_in_map;
    TrajectoryFileSummary frames;
    TrajectoryFileSummary keyframes;
};

namespace trajectory_export
{
// Shared implementation for System's existing EuRoC exports and finite fixtures.
// References are borrowed for this call only; the owner must stop all producers.
struct FrameHistory
{
    const std::list<Sophus::SE3f>& relative_poses;
    const std::list<KeyFrame*>& references;
    const std::list<double>& timestamps_sec;
    const std::list<bool>& lost;
};

// Largest number of non-bad KFs still belonging to the map; ties use lower ID.
Map* SelectMap(const std::vector<Map*>& maps);
TrajectoryFileSummary SaveFrames(const std::string& filename, Map* map,
                                const FrameHistory& history, bool inertial);
TrajectoryFileSummary SaveKeyFrames(const std::string& filename, Map* map, bool inertial);

// No data returns an explicit status. Invalid arguments/history or I/O failure
// throw; files may be partial. The caller owns staging and completion markers.
TrajectoryExportResult SaveEvaluation(bool shutdown_completed, Atlas& atlas,
                                    const FrameHistory& history, bool inertial,
                                    const std::string& frame_filename,
                                    const std::string& keyframe_filename);
}
}

#endif
