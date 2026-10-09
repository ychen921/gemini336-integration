#pragma once

#include "G2oTypes.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"

#include <array>
#include <cmath>
#include <algorithm>

namespace ORB_SLAM3
{
namespace GBADiagnostics
{
struct ObjectiveCategory
{
    std::size_t count = 0;
    double raw = 0, robust = 0;
    std::size_t nonfinite = 0;
};

struct ObjectiveBreakdown
{
    // OTHER prevents unrecognized edges from silently disappearing from totals.
    std::array<ObjectiveCategory, 6> categories;
    double robust_sum = 0;
};

inline ObjectiveBreakdown SummarizeObjective(const g2o::SparseOptimizer& optimizer)
{
    ObjectiveBreakdown result;
    for(const g2o::OptimizableGraph::Edge* edge : optimizer.activeEdges())
    {
        std::size_t category = 5;
        if(dynamic_cast<const EdgeMono*>(edge)) category = 0;
        else if(dynamic_cast<const EdgeStereo*>(edge)) category = 1;
        else if(dynamic_cast<const EdgeInertial*>(edge)) category = 2;
        else if(dynamic_cast<const EdgeGyroRW*>(edge)) category = 3;
        else if(dynamic_cast<const EdgeAccRW*>(edge)) category = 4;

        const double raw = edge->chi2();
        double robust = raw;
        if(edge->robustKernel())
        {
            Eigen::Vector3d rho;
            edge->robustKernel()->robustify(raw, rho);
            robust = rho[0];
        }
        ObjectiveCategory& totals = result.categories[category];
        ++totals.count;
        totals.raw += raw;
        totals.robust += robust;
        if(!std::isfinite(raw) || !std::isfinite(robust))
            ++totals.nonfinite;
    }
    for(const ObjectiveCategory& category : result.categories)
        result.robust_sum += category.robust;
    return result;
}

inline bool ObjectiveSumMatches(double sum, double reference)
{
    // Grouping changes summation order. Do not interpret rounding as a mismatch,
    // and never claim a match for non-finite objectives.
    return std::isfinite(sum) && std::isfinite(reference) &&
           std::abs(sum - reference) <= 1e-8 * std::max(1.0, std::abs(reference));
}

inline bool TrialIsAnomalous(const g2o::OptimizationAlgorithmLevenberg::TrialDiagnostic& trial)
{
    return (trial.accepted && trial.evaluatedChi > trial.currentChi) || trial.scale <= 0 ||
           !trial.linearSolveOK || !std::isfinite(trial.currentChi) ||
           !std::isfinite(trial.tempChi) || !std::isfinite(trial.evaluatedChi) ||
           !std::isfinite(trial.scale) || !std::isfinite(trial.rho) ||
           !std::isfinite(trial.lambda) || !std::isfinite(trial.nextLambda);
}
} // namespace GBADiagnostics
} // namespace ORB_SLAM3
