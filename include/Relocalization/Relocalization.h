//
// Created by bing on 2023/6/7.
//

#ifndef ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H
#define ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H

#include "KeyFrame.h"
//#include "DBoW2/DBoW2.h"
#include "R_ORBmatcher.h"
#include "R_Frame.h"
#include "Tracking.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include "R_Optimizer.h"
#include "MapLine.h"
//#include "LocalMapping.h"

namespace ORB_SLAM3 {
typedef std::pair<cv::Mat, cv::Mat> R_pose;
typedef std::pair<R_pose, double> R_Pose_KF;


class R_Frame;
//class KeyFrame;
class Tracking;
class KeyFrameDatabase;
class Converter;
class MapLine;
class LocalMapping;
class Relocalization {
public:
    Relocalization(const std::string &strSettingPath );

    std::vector<R_Frame> LoadImages(const std::string &strimagePath, ORBVocabulary* vocab, ORBextractor* orbextractor);


//    Relocalization(const std::string &vocpath, const std::string &imgpath);

    void InsertKeyFrame(KeyFrame* pKF);

    void ClearKF();

    bool ParseCamParamFile(cv::FileStorage &fSettings);

    void Run();

    void add(R_Frame* R_F);

    void UpdatePose(Map* Cur_Map);

    void UpdatePose2(Map* Cur_Map);

    void UpdatePose3(Map* Cur_Map);

    std::map<double, R_pose> GetAllPose();

//    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);


    vector<R_Frame*> DetectRelocalization(KeyFrame* pKF);


    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    void ResetIfRequested();

private:
//    DBoW2::Database m_db;
    ORBVocabulary* m_vocab;
//    KeyFrameDatabase* m_KeyFrameDatabase;
    std::vector<list<R_Frame*>> m_InvertedFile_R;

    cv::Mat m_R_Tcr;   // 重定位中词带帧到当前关键帧的位姿变换
    cv::Mat m_R_T21;   // 当前SLAM的世界坐标系到先验地图的世界坐标系的变换
    cv::Mat m_R_T12;   // 先验地图的世界坐标系到当前SLAM的世界坐标系的变换

    long unsigned int m_R_id = 0;  // 上一次发生重定位的关键帧ID

protected:
    std::mutex mMutexRelocalQueue;
    std::list<KeyFrame*> mlRelocalKeyFrames;

    std::string m_vocPath;
    std::string m_imgPath;

    std::mutex mMutexReset;
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;

    bool CheckNewKeyFrames();

    std::mutex mMutexNewKFs;   // 读取从Tracking线程中来的关键帧
    std::mutex mMutexRKFS;     // 检测到重定位后进行关键帧的位姿恢复
//
//    KeyFrame* m_CurrentKeyFrame;

    std::list<KeyFrame*> lKFs;

    std::map<double, R_pose> mp_R_pose;  // 保存最后调整后的位姿（先验地图的世界坐标系）

//    Tracking* mp_R_Tracker;

    LocalMapping *mp_R_LocalMapper;


public:
    ORBextractor* orb_exetractor;

    std::vector<R_Frame> m_Vec_BowF;

//    std::vector<cv::Point2f> m_PrevMatched;
//
//    std::vector<int> m_IniMatches;

};
}

#endif //ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H
