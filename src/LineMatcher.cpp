/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "LineMatcher.h"

//STL
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "gridStructure.h"
#include "Converter.h"

namespace ORB_SLAM3 {
    const int LineMatcher::TH_HIGH = 100;
    const int LineMatcher::TH_LOW = 50;

int LineMatcher::matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    std::vector<std::vector<cv::DMatch>> matches_;
    cv::Ptr<cv::BFMatcher> bfm = cv::BFMatcher::create(cv::NORM_HAMMING, false); // cross-check
    bfm->knnMatch(desc1, desc2, matches_, 2);

    if (desc1.rows != matches_.size())
        throw std::runtime_error("[matchNNR] Different size for matches and descriptors!");

    for (int idx = 0, nsize = desc1.rows; idx < nsize; ++idx) {
        if (matches_[idx][0].distance < matches_[idx][1].distance * nnr) {
            matches_12[idx] = matches_[idx][0].trainIdx;
            matches++;
        }
    }

    return matches;
}

int LineMatcher::match(const std::vector<MapLine*> &vpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12)
{

    cv::Mat desc1;
    desc1.reserve(vpLocalMapLines.size());
    for(int i=0,z=vpLocalMapLines.size(); i<z; ++i)
        desc1.push_back(vpLocalMapLines[i]->GetDescriptor());
    cv::Mat desc2;
    CurrentFrame.mDescriptors_Line.copyTo(desc2);

    bool bestLRMatches = true; // true if double-checking the matches between the two images
    int matches;
    if (bestLRMatches) {
        std::vector<int> matches_21;
        matches = matchNNR(desc1, desc2, nnr, matches_12);
        matchNNR(desc2, desc1, nnr, matches_21);
        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }
    else matches = matchNNR(desc1, desc2, nnr, matches_12);

    return matches;
}

int LineMatcher::match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    bool bestLRMatches = true; // true if double-checking the matches between the two images
    if (bestLRMatches) {
        int matches;
        std::vector<int> matches_21;
        matches = matchNNR(desc1, desc2, nnr, matches_12);
        matchNNR(desc2, desc1, nnr, matches_21);
        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }

        return matches;
    } else
        return matchNNR(desc1, desc2, nnr, matches_12);
}

int LineMatcher::SerachForInitialize(Frame &InitialFrame, Frame &CurrentFrame,
                                     std::vector<std::pair<int, int>> &LineMatches) {
    LineMatches.clear();
    int nmatches = 0;
    BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
    Mat ldesc1, ldesc2;
    vector<vector<DMatch>> lmatches;
    ldesc1 = InitialFrame.mDescriptors_Line;
    ldesc2 = CurrentFrame.mDescriptors_Line;
    bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

    double nn_dist_th, nn12_dist_th;
    CurrentFrame.lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
    nn12_dist_th = nn12_dist_th*0.5;
    sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
    for(int i=0; i<lmatches.size(); i++)
    {
        int qdx = lmatches[i][0].queryIdx;
        int tdx = lmatches[i][0].trainIdx;
        double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
        if(dist_12>nn12_dist_th)
        {
            LineMatches.push_back(make_pair(qdx, tdx));
            nmatches++;
        }
    }
    return nmatches;
}

int LineMatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                                        std::vector<std::pair<size_t, size_t>> &vMatchedPairs) {
    vMatchedPairs.clear();
    int nmatches = 0;
    BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
    cv::Mat ldesc1, ldesc2;
    vector<vector<DMatch>> lmatches;
    ldesc1 = pKF1->mDescriptors_l;
    ldesc2 = pKF2->mDescriptors_l;
    bfm->knnMatch(ldesc1, ldesc2, lmatches, 2);

    double nn_dist_th, nn12_dist_th;
    pKF1->lineDescriptorMAD(lmatches, nn_dist_th, nn12_dist_th);
    nn12_dist_th = nn12_dist_th * 0.1;
    sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
    for (int i = 0; i < lmatches.size(); i++) {
        int qdx = lmatches[i][0].queryIdx;
        int tdx = lmatches[i][0].trainIdx;

        if (pKF1->GetMapLine(qdx) || pKF2->GetMapLine(tdx))
            continue;

        double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
        if (dist_12 > nn12_dist_th) {
            vMatchedPairs.push_back(make_pair(qdx, tdx));
            nmatches++;
        }
    }
    return nmatches;
}

int LineMatcher::distance(const cv::Mat &a, const cv::Mat &b) {

    // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;
    for(int i = 0; i < 8; i++, pa++, pb++) {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

int LineMatcher::matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1,
              const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2,
              const GridWindow &w,
              std::vector<int> &matches_12) {

    if (lines1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each line needs a corresponding descriptor!");

    double lineSimTh = 0.75;   // threshold for cosine similarity
    double minRatio12L = 0.9;  // parameter to avoid outliers in line matching
    bool bestLRMatches = true; // true if double-checking the matches between the two images
    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    int best_d, best_d2, best_idx;
    std::vector<int> matches_21, distances;
    if (bestLRMatches) {
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0, nsize = lines1.size(); i1 < nsize; ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const line_2d &coords = lines1[i1];
        cv::Mat desc = desc1.row(i1);

        const point_2d sp = coords.first;
        const point_2d ep = coords.second;

        std::pair<double, double> v = std::make_pair(ep.first - sp.first, ep.second - sp.second);
        normalize(v);

        std::unordered_set<int> candidates;
        grid.get(sp.first, sp.second, w, candidates);
        grid.get(ep.first, ep.second, w, candidates);

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {
            if (i2 < 0 || i2 >= desc2.rows) continue;

            if (std::abs(dot(v, directions2[i2])) < lineSimTh)
                continue;

            const int d = distance(desc, desc2.row(i2));

            if (bestLRMatches) {
                if (d < distances[i2]) {
                    distances[i2] = d;
                    matches_21[i2] = i1;
                } else continue;
            }

            if (d < best_d) {
                best_d2 = best_d;
                best_d = d;
                best_idx = i2;
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * minRatio12L) {
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (bestLRMatches) {
        for (int i1 = 0, nsize = matches_12.size(); i1 < nsize; ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

int LineMatcher::SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const GridStructure &grid, const float &th, const float &angth)
{
    int matches = 0;
    const int TH_HIGH = 120;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    for(int i=0; i<LastFrame.N_l; i++)
    {
        MapLine* pML = LastFrame.mvpMapLines[i];
        if(pML)
        {
            if(!LastFrame.mvbOutlier_Line[i])
            {
                // Project Lines to the Image
                Eigen::Vector3d sp = pML->GetWorldPos().head(3);
                Eigen::Vector3d ep = pML->GetWorldPos().tail(3);
                cv::Mat x3Dw_sp = Converter::toCvMat(sp);
                cv::Mat x3Dw_ep = Converter::toCvMat(ep);
                cv::Mat x3Dc_sp = Rcw*x3Dw_sp+tcw;
                cv::Mat x3Dc_ep = Rcw*x3Dw_ep+tcw;

                const float invzc_sp = 1.0/x3Dc_sp.at<float>(2);
                const float invzc_ep = 1.0/x3Dc_ep.at<float>(2);

                if(invzc_sp<0 || invzc_ep<0)
                    continue;

                cv::Point2f uv_sp = CurrentFrame.mpCamera->project(x3Dc_sp);
                cv::Point2f uv_ep = CurrentFrame.mpCamera->project(x3Dc_ep);

                if(uv_sp.x<CurrentFrame.mnMinX || uv_sp.x>CurrentFrame.mnMaxX || uv_ep.x<CurrentFrame.mnMinX || uv_ep.x>CurrentFrame.mnMaxX)
                    continue;
                if(uv_sp.y<CurrentFrame.mnMinY || uv_sp.y>CurrentFrame.mnMaxY || uv_ep.y<CurrentFrame.mnMinY || uv_ep.y>CurrentFrame.mnMaxY)
                    continue;

                int nLastOctave = LastFrame.mvKeys_Line[i].octave;

                // Search in a window. Size depends on scale
                int window = floor(th);

                if(CurrentFrame.mvScaleFactors_l[nLastOctave]>1)
                    window = floor(th+CurrentFrame.mvScaleFactors_l[nLastOctave]);

                GridWindow win;
                win.width = std::make_pair(window, window);
                win.height = std::make_pair(window, window);

                const line_2d coords = std::make_pair(std::make_pair(uv_sp.x * CurrentFrame.inv_width, uv_sp.y * CurrentFrame.inv_height),
                                        std::make_pair(uv_ep.x * CurrentFrame.inv_width, uv_ep.x * CurrentFrame.inv_height)); 

                const point_2d spoint = coords.first;
                const point_2d epoint = coords.second;

                std::unordered_set<int> candidates;
                grid.get(spoint.first, spoint.second, win, candidates);
                grid.get(epoint.first, epoint.second, win, candidates);

                if (candidates.empty()) continue;

                const cv::Mat dML = pML->GetDescriptor();

                int bestDist = 256;
                int bestidx = -1;

                for (const int &i2 : candidates) 
                {
                    if(CurrentFrame.mvpMapLines[i2])
                        if(CurrentFrame.mvpMapLines[i2]->Observations()>0)
                            continue;

                    const int dist = distance(dML, CurrentFrame.mDescriptors_Line.row(i2));

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestidx=i2;
                    }
                }

                if(bestDist<=TH_HIGH)
                {
                    float theta = CurrentFrame.mvKeysUn_Line[bestidx].angle-atan2(uv_ep.y - uv_sp.y, uv_ep.x - uv_sp.x);;
                    if(theta<-M_PI) theta+=2*M_PI;
                    else if(theta>M_PI) theta-=2*M_PI;
                    if(fabs(theta)<angth)
                    {
                        CurrentFrame.mvpMapLines[bestidx]=pML;
                        matches++;
                    }
                }

            }
        }
    }
    return matches;
}

int LineMatcher::Fuse(KeyFrame *pKF, const vector<MapLine *> &vpMapLines, const float th) {
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();

    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;

    cv::Mat Ow = pKF->GetCameraCenter();
    int nFused = 0;
    const int nLines = vpMapLines.size();

    // For each candidate MapLine project and match
    for (int iML = 0; iML < nLines; iML++) {
        MapLine* pML = vpMapLines[iML];
        if (!pML || pML->isBad())
            continue;

        Vector6d P = pML->GetWorldPos();

        cv::Mat SP = (cv::Mat_<float>(3,1) << P(0), P(1), P(2));
        cv::Mat EP = (cv::Mat_<float>(3,1) << P(3), P(4), P(5));

        const cv::Mat SPc = Rcw * SP + tcw;
        const auto &SPcX = SPc.at<float>(0);
        const auto &SPcY = SPc.at<float>(1);
        const auto &SpcZ = SPc.at<float>(2);

        const cv::Mat EPc = Rcw * EP + tcw;
        const auto &EPcX = EPc.at<float>(0);
        const auto &EPcY = EPc.at<float>(1);
        const auto &EPcZ = EPc.at<float>(2);

        if (SpcZ < 0.0f || EPcZ < 0.0f)
            continue;

        const float invz1 = 1.0f / SpcZ;
        const float u1 = fx * SPcX * invz1 + cx;
        const float v1 = fy * SPcY * invz1 + cy;

        if (u1 < pKF->mnMinX || u1 > pKF->mnMaxX)
            continue;
        if (v1 < pKF->mnMinY || v1 > pKF->mnMaxY)
            continue;

        const float invz2 = 1.0f / EPcZ;
        const float u2 = fx * EPcX * invz2 + cx;
        const float v2 = fy * EPcY * invz2 + cy;

        if (u2 < pKF->mnMinX || u2 > pKF->mnMaxX)
            continue;
        if (v2 < pKF->mnMinY || v2 > pKF->mnMaxY)
            continue;

        const float maxDistance = pML->GetMaxDistanceInvariance();
        const float minDistance = pML->GetMinDistanceInvariance();

        const cv::Mat OM = 0.5 * (SP + EP) - Ow;
        const float dist = cv::norm(OM);

        if (dist < minDistance || dist > maxDistance)
            continue;

        cv::Mat pn = pML->GetNormal();
        if (OM.dot(pn) < 0.5 * dist)
            continue;

        const int nPredictedLevel = pML->PredictScale(dist,pKF->mfLogScaleFactor);
        const float radius = th * pKF->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices = pKF->GetLinesInArea(u1,v1,u2,v2,radius);

        if (vIndices.empty())
            continue;
        const cv::Mat dML = pML->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;

        for (unsigned long idx : vIndices) {
            const int &klLevel = pKF->mvKeys_Line[idx].octave;
            if (klLevel < nPredictedLevel - 1 || klLevel > nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors_l.row(idx);
            const int dist = DescriptorDistance(dML, dKF);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if (bestDist <= TH_LOW) {
            MapLine* pMLinKF = pKF->GetMapLine(bestIdx);
            if (pMLinKF) {
                if (!pMLinKF->isBad()) {
                    if (pMLinKF->Observations() > pML->Observations())
                        pML->Replace(pMLinKF);
                    else
                        pMLinKF->Replace(pML);
                }
            } else {
                pML->AddObservation(pKF,bestIdx);
                pKF->AddMapLine(pML,bestIdx);
            }
            nFused++;
        }
    }
//    cout << "Fused : " << nFused << " lines " << endl;
    return nFused;
}

int LineMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;
    for (int i = 0; i < 8; i++, pa++, pb++) {
        unsigned int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v +(v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 25;
    }
    return dist;
}

} //namesapce ORB_SLAM3
