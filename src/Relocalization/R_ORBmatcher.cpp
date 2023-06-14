//
// Created by bing on 2023/6/12.
//

#include "R_ORBmatcher.h"

namespace ORB_SLAM3 {

const int R_ORBmatcher::TH_HIGH = 100;
const int R_ORBmatcher::TH_LOW = 50;
const int R_ORBmatcher::HISTO_LENGTH = 30;

R_ORBmatcher::R_ORBmatcher(float nnratio, bool checkOri) : m_nnratio(nnratio), m_checkOri(checkOri){}

int R_ORBmatcher::SearchForRelocalization(R_Frame &F1, KeyFrame *F2, std::vector<cv::Point2f> &vbPrevMatched,
                                          std::vector<int> &vnMatches12, int windowSize) {
    int nmatches = 0;
    vnMatches12 = vector<int>(F1.m_keypoints.size(), -1);
    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++) {
        rotHist[i].reserve(500);
    }
    const float factor = 1.0f / HISTO_LENGTH;
    std::vector<int> vMatchedDistance(F2->mvKeysUn.size(), INT_MAX);
    std::vector<int> vnMatches21(F2->mvKeysUn.size(), -1);

    for (size_t i1 = 0, iend1 = F1.m_keypoints.size(); i1 < iend1; i1++) {
        cv::KeyPoint kp1 = F1.m_keypoints[i1];
        int level1 = kp1.octave;
        if (level1 > 0)
            continue;

        vector<size_t> vIndices2 = F2->GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize);
        if (vIndices2.empty())
            continue;
        cv::Mat d1 = F1.m_des.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;
        for (vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
            size_t i2 = *vit;
            cv::Mat d2 = F2->mDescriptors.row(i2);
            int dist = ORBmatcher::DescriptorDistance(d1, d2);
            if (vMatchedDistance[i2] <= dist)
                continue;
            if (dist < bestDist) {
                bestDist2 = bestDist;
                bestDist = dist;
                bestIdx2 = i2;
            } else if (dist < bestDist2) {
                bestDist2 = dist;
            }
        }

        if (bestDist <= TH_LOW) {
            if (bestDist <= (float)bestDist2*m_nnratio) {
                if (vnMatches21[bestIdx2] >= 0) {
                    vnMatches12[vnMatches21[bestIdx2]] = -1;
                    nmatches--;
                }
                vnMatches12[-1] = bestIdx2;
                vnMatches21[bestIdx2] = i1;
                vMatchedDistance[bestIdx2] = bestDist;
                nmatches++;

                if (m_checkOri) {
                    float rot = F1.m_keypoints[i1].angle - F2->mvKeysUn[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = round(rot * factor);
                    if (bin == HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }
    }

    if (m_checkOri) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        R_ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);
        for (int i = 0; i < HISTO_LENGTH; i++) {
            if (i == ind1 || i == ind2 || i == ind3)
                continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                int idx1 = rotHist[i][j];
                if (vnMatches12[idx1] >= 0) {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            }
        }
    }

    for (size_t i1 = 0, iend = vnMatches12.size(); i1 < iend; i1++) {
        if (vnMatches12[i1] >= 0) {
            vbPrevMatched[i1] = F2->mvKeysUn[vnMatches12[i1]].pt;
        }
    }
    return nmatches;

}

void R_ORBmatcher::R_ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3) {
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

int R_ORBmatcher::SearchForRelocalizationByOpenCV(R_Frame &F1, KeyFrame *F2) {
    // 使用汉明距离计算特征点描述子之间的距离
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(F1.m_des, F2->mDescriptors, matches, 2);

    // 进行比值测试，保留最近匹配点与次近匹配点的距离比小于一定阈值的匹配点
    const float ratio_thresh = 0.75f;
    std::vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < ratio_thresh * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }

    cout << "gmatches = " << good_matches.size() << endl;
    if (good_matches.size() <= 10) return good_matches.size();
    std::vector<cv::Point2f> points1, points2;
    for (size_t i = 0; i < good_matches.size(); i++) {
        points1.push_back(F1.m_keypoints[good_matches[i].queryIdx].pt);
        points2.push_back(F2->mvKeysUn[good_matches[i].trainIdx].pt);
    }
    cout << "points1,2 size = " << points1.size() << " " << points2.size() << endl;
    std::vector<uchar> inliers(points1.size());
    // Camera.fx: 458.654
    // Camera.fy: 457.296
    // Camera.cx: 367.215
    // Camera.cy: 248.375
    cv::Point2d principal_point(367.215,248.375); //相机光心
    float focal_length = 458.0;
    cv::Mat essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);

    cv::Mat R, t;
    cv::recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    cout << "p2p R = \n" << R << endl;
    cout << "p2p t = \n" << t << endl;

    cv::Mat Tcr = cv::Mat::eye(4,4,CV_32F);

    R.copyTo(Tcr.rowRange(0,3).colRange(0,3));
    t.copyTo(Tcr.rowRange(0,3).col(3));
    cout << "p2p Tcr = \n" << Tcr << endl;
    F1.SetTcr(Tcr);


    F1.m_orbmatches = good_matches;
    return good_matches.size();
}
}

