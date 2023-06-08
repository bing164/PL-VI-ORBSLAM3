//
// Created by bing on 2023/6/7.
//

#ifndef ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H
#define ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H

#include "KeyFrame.h"
//#include "DBoW2/DBoW2.h"
#include "R_Frame.h"
#include "Tracking.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM3 {

class R_Frame;
class KeyFrame;
class Tracking;
class Relocalization {
public:
    Relocalization(const std::string &strSettingPath);

//    Relocalization(const std::string &vocpath, const std::string &imgpath);

    void InsertKeyFrame(KeyFrame* pKF);

    void ClearKF();

    bool ParseCamParamFile(cv::FileStorage &fSettings);

    void Run();


    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    void ResetIfRequested();

public:
//    DBoW2::Database m_db;
    ORBVocabulary* m_vocab;

protected:
    std::mutex mMutexRelocalQueue;
    std::list<KeyFrame*> mlRelocalKeyFrames;

    std::string m_vocPath;
    std::string m_imgPath;

    std::mutex mMutexReset;
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;

    bool CheckNewKeyFrames();

};
}

#endif //ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H
