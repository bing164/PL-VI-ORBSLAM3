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

namespace ORB_SLAM3 {

class R_Frame;
//class KeyFrame;
class Tracking;
class KeyFrameDatabase;
class Converter;

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

    vector<R_Frame*> DetectRelocalization(KeyFrame* pKF);


    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    void ResetIfRequested();

private:
//    DBoW2::Database m_db;
    ORBVocabulary* m_vocab;
//    KeyFrameDatabase* m_KeyFrameDatabase;
    std::vector<list<R_Frame*>> m_InvertedFile_R;

protected:
    std::mutex mMutexRelocalQueue;
    std::list<KeyFrame*> mlRelocalKeyFrames;

    std::string m_vocPath;
    std::string m_imgPath;

    std::mutex mMutexReset;
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;

    bool CheckNewKeyFrames();

    std::mutex mMutexNewKFs;
//
//    KeyFrame* m_CurrentKeyFrame;

public:
    ORBextractor* orb_exetractor;

    std::vector<R_Frame> m_Vec_BowF;

//    std::vector<cv::Point2f> m_PrevMatched;
//
//    std::vector<int> m_IniMatches;

};
}

#endif //ORB_SLAM3_RELOCALIZATION_RELOCALIZATION_H
