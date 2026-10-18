/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Sim3Solver.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM3
{

namespace
{
bool HasNonDegenerateTriangle(const Eigen::Matrix3f& points)
{
    // Double intermediates keep the geometry check safe across the finite float
    // range. Area relative to the longest edge avoids a threshold in metres.
    const Eigen::Vector3d edge01 = points.col(1).cast<double>() - points.col(0).cast<double>();
    const Eigen::Vector3d edge02 = points.col(2).cast<double>() - points.col(0).cast<double>();
    const double longestEdgeSquared = std::max(edge01.squaredNorm(),
        std::max(edge02.squaredNorm(), (edge02 - edge01).squaredNorm()));
    if(!(longestEdgeSquared > 0.0)) return false;

    const double relativeAreaSquared = edge01.cross(edge02).squaredNorm() /
        (longestEdgeSquared * longestEdgeSquared);
    // Collinear and float-precision ill-conditioned triangles cannot constrain
    // a stable rotation. A non-collinear planar triangle remains valid.
    return std::isfinite(relativeAreaSquared) &&
        relativeAreaSquared > static_cast<double>(std::numeric_limits<float>::epsilon());
}
} // namespace


Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale,
                       vector<KeyFrame*> vpKeyFrameMatchedMP):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale),
    pCamera1(pKF1->mpCamera), pCamera2(pKF2->mpCamera)
{
    bool bDifferentKFs = false;
    if(vpKeyFrameMatchedMP.empty())
    {
        bDifferentKFs = true;
        vpKeyFrameMatchedMP = vector<KeyFrame*>(vpMatched12.size(), pKF2);
    }

    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mN1 = vpMatched12.size();

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    Eigen::Matrix3f Rcw1 = pKF1->GetRotation();
    Eigen::Vector3f tcw1 = pKF1->GetTranslation();
    Eigen::Matrix3f Rcw2 = pKF2->GetRotation();
    Eigen::Vector3f tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    size_t idx=0;

    KeyFrame* pKFm = pKF2; //Default variable
    for(int i1=0; i1<mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            if(bDifferentKFs)
                pKFm = vpKeyFrameMatchedMP[i1];

            int indexKF1 = get<0>(pMP1->GetIndexInKeyFrame(pKF1));
            int indexKF2 = get<0>(pMP2->GetIndexInKeyFrame(pKFm));

            if(indexKF1<0 || indexKF2<0)
                continue;

            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKFm->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKFm->mvLevelSigma2[kp2.octave];

            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            Eigen::Vector3f X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            Eigen::Vector3f X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    FromCameraToImage(mvX3Dc1,mvP1im1,pCamera1);
    FromCameraToImage(mvX3Dc2,mvP2im2,pCamera2);

    SetRansacParameters();
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // number of correspondences

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mnIterations = 0;
}

Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<3 || N<mRansacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // Rejected samples still consume the iteration budget, but must not
        // reuse a previous model in CheckInliers or update the best model.
        if(!ComputeSim3(P3Dc1i,P3Dc2i)) continue;

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i;
            mBestRotation = mR12i;
            mBestTranslation = mt12i;
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return Eigen::Matrix4f::Identity();
}

Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge)
{
    bNoMore = false;
    bConverge = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<3 || N<mRansacMinInliers)
    {
        bNoMore = true;
        return Eigen::Matrix4f::Identity();
    }

    vector<size_t> vAvailableIndices;

    Eigen::Matrix3f P3Dc1i;
    Eigen::Matrix3f P3Dc2i;

    int nCurrentIterations = 0;

    // All samples may be rejected; convergence/inlier flags identify this
    // placeholder as a failed search rather than an identity-pose solution.
    Eigen::Matrix4f bestSim3 = Eigen::Matrix4f::Identity();

    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            P3Dc1i.col(i) = mvX3Dc1[idx];
            P3Dc2i.col(i) = mvX3Dc2[idx];

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        if(!ComputeSim3(P3Dc1i,P3Dc2i)) continue;

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i;
            mBestRotation = mR12i;
            mBestTranslation = mt12i;
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                bConverge = true;
                return mBestT12;
            }
            else
            {
                bestSim3 = mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return bestSim3;
}

Eigen::Matrix4f Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C)
{
    C = P.rowwise().sum();
    C = C / P.cols();
    for(int i=0; i<P.cols(); i++)
    Pr.col(i) = P.col(i) - C;
}


bool Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Invalid RANSAC draws are expected and deliberately produce no per-sample
    // log or synchronous write. Let the caller continue with another draw.
    if(!P1.allFinite() || !P2.allFinite() ||
       !HasNonDegenerateTriangle(P1) || !HasNonDegenerateTriangle(P2))
        return false;

    // Step 1: Centroid and relative coordinates

    Eigen::Matrix3f Pr1; // Relative coordinates to centroid (set 1)
    Eigen::Matrix3f Pr2; // Relative coordinates to centroid (set 2)
    Eigen::Vector3f O1; // Centroid of P1
    Eigen::Vector3f O2; // Centroid of P2

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);
    if(!O1.allFinite() || !O2.allFinite() || !Pr1.allFinite() || !Pr2.allFinite())
        return false;

    // Step 2: Compute M matrix

    Eigen::Matrix3f M = Pr2 * Pr1.transpose();
    // Finite inputs can still overflow or underflow during float products.
    if(!M.allFinite() || !(M.cwiseAbs().maxCoeff() > 0.0f)) return false;

    // Step 3: Compute N matrix
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    Eigen::Matrix4f N;

    N11 = M(0,0)+M(1,1)+M(2,2);
    N12 = M(1,2)-M(2,1);
    N13 = M(2,0)-M(0,2);
    N14 = M(0,1)-M(1,0);
    N22 = M(0,0)-M(1,1)-M(2,2);
    N23 = M(0,1)+M(1,0);
    N24 = M(2,0)+M(0,2);
    N33 = -M(0,0)+M(1,1)-M(2,2);
    N34 = M(1,2)+M(2,1);
    N44 = -M(0,0)-M(1,1)+M(2,2);

    N << N11, N12, N13, N14,
         N12, N22, N23, N24,
         N13, N23, N33, N34,
         N14, N24, N34, N44;
    if(!N.allFinite()) return false;


    // Step 4: Eigenvector of the highest eigenvalue
    Eigen::EigenSolver<Eigen::Matrix4f> eigSolver;
    eigSolver.compute(N);
    // Eigen outputs are not usable after a failed decomposition.
    if(eigSolver.info() != Eigen::Success) return false;

    Eigen::Vector4f eval = eigSolver.eigenvalues().real();
    Eigen::Matrix4f evec = eigSolver.eigenvectors().real(); //evec[0] is the quaternion of the desired rotation
    if(!eval.allFinite() || !evec.allFinite()) return false;

    int maxIndex; // should be zero
    eval.maxCoeff(&maxIndex);

    // Eigen returns w,x,y,z. Direct conversion handles both signs of an
    // identity quaternion without dividing by its zero imaginary-part norm.
    Eigen::Quaternionf quaternion(evec(0,maxIndex), evec(1,maxIndex),
                                  evec(2,maxIndex), evec(3,maxIndex));
    const float quaternionNorm = quaternion.norm();
    if(!std::isfinite(quaternionNorm) || !(quaternionNorm > 0.0f)) return false;
    quaternion.normalize();
    const Eigen::Matrix3f rotation = quaternion.toRotationMatrix();
    if(!rotation.allFinite()) return false;

    // Step 5: Rotate set 2
    const Eigen::Matrix3f P3 = rotation*Pr2;
    if(!P3.allFinite()) return false;

    // Step 6: Scale

    float scale = 1.0f;
    if(!mbFixScale)
    {
        // Keep the scale calculation free of per-draw logging and diagnostic
        // OpenCV allocations, including draws rejected by the range checks.
        const double nom = (Pr1.array() * P3.array()).sum();
        const Eigen::Array<float,3,3> aux_P3 = P3.array() * P3.array();
        const double den = aux_P3.sum();

        if(!std::isfinite(nom) || !std::isfinite(den) || !(den > 0.0)) return false;
        scale = static_cast<float>(nom/den);
    }
    // The inverse transform requires a positive, finite, invertible scale.
    if(!std::isfinite(scale) || !(scale > 0.0f) || !std::isfinite(1.0f/scale)) return false;

    // Step 7: Translation
    const Eigen::Vector3f translation = O1 - scale * rotation * O2;

    // Step 8: Transformation

    // Step 8.1 T12
    Eigen::Matrix4f transform12 = Eigen::Matrix4f::Identity();

    const Eigen::Matrix3f sR = scale*rotation;
    transform12.block<3,3>(0,0) = sR;
    transform12.block<3,1>(0,3) = translation;


    // Step 8.2 T21
    Eigen::Matrix4f transform21 = Eigen::Matrix4f::Identity();
    const Eigen::Matrix3f sRinv = (1.0f/scale)*rotation.transpose();

    transform21.block<3,3>(0,0) = sRinv;

    const Eigen::Vector3f tinv = -sRinv * translation;
    transform21.block<3,1>(0,3) = tinv;
    if(!translation.allFinite() || !transform12.allFinite() || !transform21.allFinite())
        return false;

    // Publish only a fully validated model. A rejected draw cannot partially
    // overwrite the state retained from the last successful computation.
    mR12i = rotation;
    ms12i = scale;
    mt12i = translation;
    mT12i = transform12;
    mT21i = transform21;
    return true;
}


void Sim3Solver::CheckInliers()
{
    vector<Eigen::Vector2f> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,pCamera1);
    Project(mvX3Dc1,vP1im2,mT21i,pCamera2);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        Eigen::Vector2f dist1 = mvP1im1[i] - vP2im1[i];
        Eigen::Vector2f dist2 = vP1im2[i] - mvP2im2[i];

        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}

Eigen::Matrix4f Sim3Solver::GetEstimatedTransformation()
{
    return mBestT12;
}

Eigen::Matrix3f Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation;
}

Eigen::Vector3f Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation;
}

float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

void Sim3Solver::Project(const vector<Eigen::Vector3f> &vP3Dw, vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera* pCamera)
{
    Eigen::Matrix3f Rcw = Tcw.block<3,3>(0,0);
    Eigen::Vector3f tcw = Tcw.block<3,1>(0,3);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        Eigen::Vector3f P3Dc = Rcw*vP3Dw[i]+tcw;
        Eigen::Vector2f pt2D = pCamera->project(P3Dc);
        vP2D.push_back(pt2D);
    }
}

void Sim3Solver::FromCameraToImage(const vector<Eigen::Vector3f> &vP3Dc, vector<Eigen::Vector2f> &vP2D, GeometricCamera* pCamera)
{
    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        Eigen::Vector2f pt2D = pCamera->project(vP3Dc[i]);
        vP2D.push_back(pt2D);
    }
}

} //namespace ORB_SLAM
