//
// Created by bing on 2023/6/8.
//

#include "R_Frame.h"

namespace ORB_SLAM3 {

inline cv::Point2d pixel2cam(const cv::Point& p, const cv::Mat &K) {
    return cv::Point2d(
         (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
         (p.y - K.at<double>(1,2)) / K.at<double>(1,1)
    );
}

//R_Frame::R_Frame(const R_Frame &r_frame) : orb_exetractor(r_frame.orb_exetractor), m_keypoints(r_frame.m_keypoints),
//m_des(r_frame.m_des), m_orbmatches(r_frame.m_orbmatches), m_pts2d(r_frame.m_pts2d), m_pts3d(r_frame.m_pts3d),
//m_BowVector(r_frame.m_BowVector), m_FeatVec(r_frame.m_FeatVec), m_colorImgs(r_frame.m_colorImgs), m_depthImgs(r_frame.m_depthImgs),
//m_pose(r_frame.m_pose), FrameId(r_frame.FrameId), m_time(r_frame.m_time), m_K(r_frame.m_K), m_vocab(r_frame.m_vocab),
//m_InvertedFile_R(r_frame.m_InvertedFile_R), m_Tcr(r_frame.m_Tcr)
//{
//    cout << "copy !!!" << endl;
//}

R_Frame::R_Frame(const R_Frame &r_frame) : orb_exetractor(r_frame.orb_exetractor), m_keypoints(r_frame.m_keypoints),
m_des(r_frame.m_des), m_orbmatches(r_frame.m_orbmatches), m_pts2d(r_frame.m_pts2d), m_pts3d(r_frame.m_pts3d),
m_depthImgs(r_frame.m_depthImgs), m_K(r_frame.m_K), m_Tcr(r_frame.m_Tcr), m_pose(r_frame.m_pose)
//m_BowVector(r_frame.m_BowVector), m_FeatVec(r_frame.m_FeatVec), m_colorImgs(r_frame.m_colorImgs),
//m_pose(r_frame.m_pose), FrameId(r_frame.FrameId), m_time(r_frame.m_time), m_K(r_frame.m_K)
{
    cout << "copy !!!" << endl;
}


R_Frame::R_Frame(const cv::Mat &colorImgs, const cv::Mat &depthImgs, const Eigen::Isometry3d pose,
                 int i, const std::string &time, cv::Mat K, ORBextractor* orb, ORBVocabulary* voc)
:
m_colorImgs(colorImgs), m_depthImgs(depthImgs),
m_pose(pose), m_K(K),
FrameId(i), m_time(time), orb_exetractor(orb), m_vocab(voc)
{
    std::vector<int> vLapping = {0,1000};
    (*orb_exetractor)(colorImgs, cv::Mat(), m_keypoints, m_des, vLapping);

    if (m_BowVector.empty()) {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(m_des);
        m_vocab->transform(vCurrentDesc, m_BowVector, m_FeatVec, 4);
//        cout << "size = " << m_vocab->size() << endl;
    }
//    m_KeyFrameDatabase->mvInvertedFile_R.resize(m_vocab->size());
//    for(DBoW2::BowVector::const_iterator vit= m_BowVector.begin(), vend=m_BowVector.end(); vit!=vend; vit++)
//        m_KeyFrameDatabase->mvInvertedFile_R[vit->first].push_back(this);
//    m_InvertedFile_R.resize(m_vocab->size());

}

void R_Frame::GetMapPoints(KeyFrame *Cur_F, R_Frame* Bow_F) {
    cout << "m_orbmatches = " << Bow_F->m_orbmatches.size() << endl;
    for (auto m : Bow_F->m_orbmatches) {
        unsigned int d = Bow_F->m_depthImgs.ptr<unsigned short>
                (int(Bow_F->m_keypoints[m.queryIdx].pt.y))
                [int(Bow_F->m_keypoints[m.queryIdx].pt.x)];
        if (d == 0)
            continue;

        float dd = d / 5000.0;
        cv::Point2d p1 = pixel2cam(Bow_F->m_keypoints[m.queryIdx].pt, Bow_F->m_K);
        Bow_F->m_pts3d.push_back(cv::Point3d(p1.x * dd, p1.y + dd, dd));
        Bow_F->m_pts2d.push_back(Cur_F->mvKeysUn[m.trainIdx].pt);
//        cout << "this p2 = " << Bow_F->m_pts2d << " p3 = " << m_pts3d << endl;

    }
    cout << "p2 = " << Bow_F->m_pts2d.size() << " p3 = " << Bow_F->m_pts3d.size() << endl;
}

void R_Frame::SetTcr(cv::Mat T) {
    m_Tcr = T.clone();
}

cv::Mat R_Frame::GetTcr() {
    return m_Tcr.clone();
}


}

