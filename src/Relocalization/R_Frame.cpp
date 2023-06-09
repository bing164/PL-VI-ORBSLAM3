//
// Created by bing on 2023/6/8.
//

#include "R_Frame.h"

namespace ORB_SLAM3 {

R_Frame::R_Frame(const cv::Mat &colorImgs, const cv::Mat &depthImgs, const Eigen::Isometry3d pose,
                 int i, const std::string &time, cv::Mat K, ORBextractor* orb, ORBVocabulary* voc)
:
//m_colorImgs(colorImgs), m_depthImgs(depthImgs),
m_pose(pose), m_K(K),
FrameId(i), m_time(time), orb_exetractor(orb), m_vocab(voc)
{
    std::vector<int> vLapping = {0,1000};
    (*orb_exetractor)(colorImgs, cv::Mat(), m_keypoints, m_des, vLapping);

    if (m_BowVector.empty()) {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(m_des);
        m_vocab->transform(vCurrentDesc, m_BowVector, m_FeatVec, 4);
    }
//    m_KeyFrameDatabase->mvInvertedFile_R.resize(m_vocab->size());
//    for(DBoW2::BowVector::const_iterator vit= m_BowVector.begin(), vend=m_BowVector.end(); vit!=vend; vit++)
//        m_KeyFrameDatabase->mvInvertedFile_R[vit->first].push_back(this);
//    m_InvertedFile_R.resize(m_vocab->size());


}

}

