//
// Created by bing on 2023/6/8.
//

#ifndef ORB_SLAM3_RELOCALIZATION_R_FRAME_H
#define ORB_SLAM3_RELOCALIZATION_R_FRAME_H

#include "opencv2/opencv.hpp"
#include "Eigen/Geometry"
#include "ORBextractor.h"
#include "ORBVocabulary.h"
#include "Relocalization.h"

namespace ORB_SLAM3 {

class ORBextractor;
class R_Frame {
public:
    R_Frame() {}

    R_Frame(const cv::Mat& colorImgs, const cv::Mat& depthImgs, const Eigen::Isometry3d pose,
            int i, const std::string &time, cv::Mat K, ORBextractor* orb, ORBVocabulary* voc);

    // 通过深度图恢复出当前图像上2D特征点对应的3D地图点Pc（相机坐标系下）
    static void GetMapPoints(KeyFrame* Cur_F, R_Frame* Bow_F);


public:
    ORBextractor* orb_exetractor;
    std::vector<cv::KeyPoint> m_keypoints;
    cv::Mat m_des;
    std::vector<cv::DMatch> m_orbmatches;
    std::vector<cv::Point3d> m_pts3d;     // 词带图像中的特征点对应的3D点（相机坐标系下）
    std::vector<cv::Point2d> m_pts2d;     // 当前图像中的2D点
    DBoW2::BowVector m_BowVector;
    DBoW2::FeatureVector m_FeatVec;

    cv::Mat m_colorImgs;
    cv::Mat m_depthImgs;
    Eigen::Isometry3d m_pose;   // Twc

    int FrameId = 0; // 对应文件夹中的文件名，从1开始
    std::string m_time;  // 每帧对应的时间戳，以rgb的为标准

    cv::Mat m_K;

private:
    ORBVocabulary* m_vocab;
//    KeyFrameDatabase* m_KeyFrameDatabase;
    std::vector<list<R_Frame*>>   m_InvertedFile_R;
};
}

#endif //ORB_SLAM3_RELOCALIZATION_R_FRAME_H
