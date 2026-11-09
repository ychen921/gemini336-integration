#ifndef ORB_SLAM3_MAP_OUTPUT_H
#define ORB_SLAM3_MAP_OUTPUT_H

#include <array>
#include <atomic>
#include <cstdint>
#include <exception>
#include <shared_mutex>
#include <vector>

namespace ORB_SLAM3
{
// Background mutation admission is shared between workers. Poison the output
// BEFORE releasing admission during unwinding, closing the window before the
// worker's outer catch records its original exception.
class MapOutputMutation
{
public:
    MapOutputMutation(std::shared_mutex &mutex, std::atomic<bool> &failed)
        : mLock(mutex), mFailed(failed), mExceptions(std::uncaught_exceptions()) {}
    ~MapOutputMutation()
    {
        if(std::uncaught_exceptions() > mExceptions) mFailed.store(true);
    }
    MapOutputMutation(const MapOutputMutation &) = delete;
    MapOutputMutation &operator=(const MapOutputMutation &) = delete;

private:
    std::shared_lock<std::shared_mutex> mLock;
    std::atomic<bool> &mFailed;
    const int mExceptions;
};

// Values only: no object lifetime or Eigen alignment requirements cross this API.
struct MapOutputContext
{
    std::uint64_t map_id = 0;
    std::uint64_t generation = 0;
};

struct MapOutputPose
{
    std::array<double, 3> translation{};
    std::array<double, 4> quaternion_xyzw{};
};

struct MapOutputKeyFrame
{
    std::uint64_t id = 0;
    double timestamp = 0.0;
    MapOutputPose camera_pose; // Twc of the left optical camera.
};

enum class MapOutputStatus { Ready, NoActiveMap, Unavailable };

struct MapOutputSnapshot
{
    MapOutputStatus status = MapOutputStatus::Unavailable;
    std::uint64_t map_id = 0;
    std::uint64_t generation = 0;
    std::vector<std::array<float, 3>> map_points;
    std::vector<MapOutputKeyFrame> keyframes;
};
}

#endif
