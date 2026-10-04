#ifndef VIEWER_STATE_H
#define VIEWER_STATE_H

#include <exception>
#include <mutex>

namespace ORB_SLAM3
{
// Coordinates reset admission and terminal cleanup independently of GUI APIs.
// A stop requested while dormant prevents map reads when Run subsequently starts.
class ViewerState
{
public:
    bool Start()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (mFinishRequested || mFinished) return false;
        mStarted = true;
        return true;
    }

    void RequestStop()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (!mFinished) mStopRequested = true;
    }

    bool Pause()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (mFinishRequested) return false;
        if (mStopRequested) mStopped = true;
        return mStopped;
    }

    bool IsStopped()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        // Dormancy alone is not a reset barrier: Start could follow this query.
        return mStopped || (!mStarted && mStopRequested);
    }

    void Release()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (mFinished || mFinishRequested) return;
        mStopRequested = false;
        mStopped = false;
    }

    void RequestFinish()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        mFinishRequested = true;
    }

    bool FinishRequested()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        return mFinishRequested;
    }

    void Finish()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        mFinished = true;
    }

    bool IsFinished()
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        return mFinished;
    }

    void RecordFailure(std::exception_ptr error)
    {
        const std::lock_guard<std::mutex> lock(mMutex);
        if (!mFailure) mFailure = error;
        mFinishRequested = true;
    }

    void RethrowFailure()
    {
        std::exception_ptr error;
        {
            const std::lock_guard<std::mutex> lock(mMutex);
            error = mFailure;
        }
        if (error) std::rethrow_exception(error);
    }

private:
    std::mutex mMutex;
    bool mStarted = false;
    bool mStopRequested = false;
    bool mStopped = false;
    bool mFinishRequested = false;
    bool mFinished = false;
    std::exception_ptr mFailure;
};
}
#endif
