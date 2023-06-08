//
// Created by bing on 2023/6/8.
//

#include "R_Frame.h"

namespace ORB_SLAM3 {

R_Frame::R_Frame(const cv::Mat &colorImgs, const cv::Mat &depthImgs, const Eigen::Isometry3d pose, int i, const std::string &time, cv::Mat K)
: m_colorImgs(colorImgs), m_depthImgs(depthImgs), m_pose(pose), m_K(K), FrameId(i), m_time(time)
{
    std::vector<int> vLapping = {0,1000};
    (*orb_exetractor)(colorImgs, cv::Mat(), m_keypoints, m_des, vLapping);
}

}

