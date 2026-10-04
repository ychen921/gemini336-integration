#ifndef ORB_SLAM3_FRAME_IMU_STATE_H
#define ORB_SLAM3_FRAME_IMU_STATE_H

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <stdexcept>

namespace ORB_SLAM3
{

// Tracking owns this state and the frames it protects. Producers release the
// guard before taking a map lock or waiting for another worker's reset response.
// Consumers take the map lock first, then keep a ready guard through frame update.
class FrameImuState
{
public:
    enum class State { Invalidated, Pending, Running, Completed, Unavailable, Failed };

    class Guard
    {
    public:
        Guard(Guard&&) = default;
        Guard& operator=(Guard&&) = default;

        void BeginFrame()
        {
            RequireLock();
            ++mOwner->mGeneration;
            mOwner->mFailure = nullptr;
            Publish(State::Pending);
        }

        void StartPreintegration()
        {
            RequireLock();
            if(mOwner->mState != State::Pending)
                throw std::logic_error("IMU preintegration requires a pending frame");
            Publish(State::Running);
        }

        void Complete(bool available)
        {
            RequireLock();
            if(mOwner->mState != State::Running &&
               !(mOwner->mState == State::Pending && !available))
                throw std::logic_error("IMU completion requires a live producer");
            Publish(available ? State::Completed : State::Unavailable);
        }

        void Fail(std::exception_ptr failure)
        {
            RequireLock();
            mOwner->mFailure = failure;
            Publish(State::Failed);
        }

        void Invalidate()
        {
            RequireLock();
            ++mOwner->mGeneration;
            Publish(State::Invalidated);
        }

        void WaitUntilReady()
        {
            RequireLock();
            const std::uint64_t generation = mOwner->mGeneration;
            // Shutdown does not cancel a live producer. Every producer exit must
            // publish a terminal result; a new generation cannot satisfy this wait.
            mOwner->mChanged.wait(mLock, [&]() {
                return generation != mOwner->mGeneration ||
                       (mOwner->mState != State::Pending && mOwner->mState != State::Running);
            });
            if(generation != mOwner->mGeneration || mOwner->mState == State::Invalidated)
                throw std::runtime_error("IMU frame was invalidated before update");
            if(mOwner->mState == State::Failed && mOwner->mFailure)
                std::rethrow_exception(mOwner->mFailure);
            if(mOwner->mState != State::Completed)
                throw std::runtime_error("IMU preintegration is unavailable for frame update");
            // Keep the lock: readiness and frame replacement must not race.
        }

        bool Owns(const FrameImuState& owner) const
        {
            return mOwner == &owner && mLock.owns_lock();
        }
        State GetState() const { RequireLock(); return mOwner->mState; }
        std::uint64_t Generation() const { RequireLock(); return mOwner->mGeneration; }
        void Unlock() { RequireLock(); mLock.unlock(); }

    private:
        friend class FrameImuState;
        explicit Guard(FrameImuState& owner) : mOwner(&owner), mLock(owner.mMutex) {}

        void RequireLock() const
        {
            if(!mLock.owns_lock()) throw std::logic_error("IMU state guard does not own its lock");
        }
        void Publish(State state)
        {
            mOwner->mState = state;
            mOwner->mChanged.notify_all();
        }

        FrameImuState* mOwner;
        std::unique_lock<std::mutex> mLock;
    };

    Guard Lock() { return Guard(*this); }

private:
    std::mutex mMutex;
    std::condition_variable mChanged;
    std::uint64_t mGeneration = 0;
    State mState = State::Invalidated;
    std::exception_ptr mFailure;
};

} // namespace ORB_SLAM3

#endif
