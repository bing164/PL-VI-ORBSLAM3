//
// Created by bing on 2023/6/7.
//

#include "Relocalization.h"

namespace ORB_SLAM3 {
    cv::Mat K = (cv::Mat_<double>(3,3) << 518.0, 0, 325.5, 0, 519.0, 253.5, 0, 0, 1);
    typedef pair<double, int> PDI;
    struct cmp {
        bool operator()(const PDI& a, const PDI& b) {
            return a.first > b.first;
        }
    };

    std::vector<R_Frame> Relocalization::LoadImages(const std::string &strimagePath, ORBVocabulary* vocab, ORBextractor* orbextractor) {
        std::string pose = strimagePath + "pose.txt";
        std::ifstream fin(pose);
        if (!fin) {
            std::cerr << "cannot find pose file!!!!" << std::endl;
            return {};
        }
        std::string line;
        int i = 0;
        std::vector<R_Frame> vec_Bow;
        vec_Bow.reserve(5);   /// 测试数据5张，后续需要根据需要修改
        while(std::getline(fin, line)) {
            std::string rgb_img = strimagePath + "color/" + std::to_string(i + 1) + ".png";
            std::string depth_img = strimagePath + "depth/" + std::to_string(i + 1) + ".pgm";
            cv::Mat colorImgs = cv::imread(rgb_img);
            cv::Mat depthImgs = cv::imread(depth_img, -1);

            double data[7] = {0};
            for (int k = 0; k < 7; k++) {
                fin >> data[k];
            }
            Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
            Eigen::Isometry3d T(q);
            T.pretranslate(Eigen::Vector3d(data[0], data[1], data[3]));
            std::cout << "\ni = " << i << "  Pose = \n" << T.matrix() << std::endl;

            string time = "123456";  /// 后续需要根据数据提供时间戳进行读取，这里暂时随便写的
            R_Frame currentFrame(colorImgs, depthImgs, T, i+1, time, K, orbextractor,vocab);
//            std::shared_ptr<R_Frame> currentFrame =
//                    std::make_shared<R_Frame>(colorImgs, depthImgs, T, i+1, time, K, orbextractor, vocab);
//            R_Frame* currentFrame = new R_Frame(colorImgs, depthImgs, T, i+1, time, K, orbextractor,vocab);
            cout << "111222" << endl;
            vec_Bow.push_back(currentFrame);
            cout << "22365456" << endl;
            add(&currentFrame);
            cout << "5165" << endl;
//            list_BowF.emplace_back(currentFrame);
//            DBoW2::BowVector Bow_vector;
//            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(currentFrame.m_des);
//            cout << "Vcurrent = " << vCurrentDesc.size() << endl;
//            vocab->transform(vCurrentDesc, currentFrame.m_BowVector, currentFrame.m_FeatVec, 4);
//            cout << "end vocab" << endl;
            i++;
        }
        return vec_Bow;

    }


Relocalization::Relocalization(const std::string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    bool succed = ParseCamParamFile(fSettings);
    if (succed) {
        std::cout << "Yaml read finished...." << endl;
        m_vocab = new ORBVocabulary();
////        ORBVocabulary * vocab = new ORBVocabulary();
        bool flag = m_vocab->loadFromTextFile(m_vocPath);
//        m_KeyFrameDatabase = new KeyFrameDatabase(*m_vocab);
        m_InvertedFile_R.resize(m_vocab->size());
        m_Vec_BowF = LoadImages(m_imgPath, m_vocab, orb_exetractor);
        cout << "m_Vec_BowF = " << m_Vec_BowF.size() << endl;
    }
}



//Relocalization::Relocalization(const std::string &vocpath, const std::string &imgpath) {
//    DBoW3::Vocabulary vocab(vocpath);
//    if (vocab.empty()) {
//        std::cerr << "Vocabulary does not exist." << std::endl;
//        return;
//    }
//
//    DBoW3::Database db(vocab, false, 0);
//    m_db = db;
//    LoadImages(imgpath, m_db);
//
////    while (1) {
////        if (CheckNewKeyFrames()) {
////
////        }
////    }
//}

void Relocalization::Run() {
    bool R_flag = false;
    while (1) {
        if (CheckNewKeyFrames()) {
            KeyFrame* CurKF;
            {
                unique_lock<mutex> lock(mMutexNewKFs);
                CurKF = mlRelocalKeyFrames.front();
                mlRelocalKeyFrames.pop_front();
            }

            // 将关键帧存入list中，等待后续进行位姿调整
            lKFs.push_back(CurKF);
//            cout << "list11 size = " << lKFs.size() << endl;
//            cout << "KF id = " << CurKF->mnId << endl;
//            if (m_R_id == 0 && CurKF->GetMap()->GetIniertialBA2() && (CurKF->mnId - m_R_id) > 50) {
            if (m_R_id == 0 && CurKF->GetMap()->GetIniertialBA2()) {
                priority_queue<PDI, vector<PDI>, cmp> que;
                R_ORBmatcher matcher(0.9, true);
                int k = 0;
                for (auto f : m_Vec_BowF) {
                    double score = m_vocab->score(CurKF->mBowVec, f.m_BowVector);
                    que.push(PDI(score, k));
                    if (que.size() > 3) {
                        que.pop();
                    }
                    k++;
                }

                int bestId = -1;
                int best_matches = 0;
                while (!que.empty()) {
                    int nmatches = matcher.SearchForRelocalizationByOpenCV(m_Vec_BowF[que.top().second], CurKF);
                    if (nmatches > best_matches) {
                        best_matches = nmatches;
                        bestId = que.top().second;
                    }
                    que.pop();
                }
                if (bestId == -1) continue;
//                if (best_matches < 100) continue;   /// 阈值还需要测试
                R_Frame Bow_F = m_Vec_BowF[bestId];
                R_Frame::GetMapPoints(CurKF, &Bow_F);
//                int inilers = R_Optimizer::R_PoseOptimization(&Bow_F, CurKF);
                m_R_Tcr = Bow_F.GetTcr();
                cout << "final m_R_Tcr = \n" << m_R_Tcr << endl;
                // 当前关键帧在先验地图世界坐标系下的位姿
                cv::Mat Tcw2 = m_R_Tcr * Converter::toCvMat(Bow_F.m_pose.matrix());
                cv::Mat Tw2c;
                cv::invert(Tcw2, Tw2c);
                m_R_T21 = Tw2c * CurKF->GetPose();
                cv::invert(m_R_T21, m_R_T12);

//                cout << "list22 size = " << lKFs.size() << endl;
//                std::list<KeyFrame*> R_CurKFs;
//                // 根据重定位的结果，更新关键帧的位姿
//                {
//                    unique_lock<std::mutex> lock(mMutexRKFS);
//                    R_CurKFs.assign(lKFs.begin(), lKFs.end());
//                    lKFs.clear();
//                }
//                UpdatePose(CurKF->GetMap());
//                UpdatePose2(CurKF->GetMap());
                m_R_id = CurKF->mnId;
                R_flag = true;
                lKFs.clear();
            }

            if (R_flag) {
                UpdatePose3(CurKF->GetMap());
            }

        }
        usleep(3000);
    }
}

void Relocalization::UpdatePose(Map* Cur_Map) {
    cv::Mat R_R21, R_t21;
    m_R_T21.rowRange(0,3).colRange(0,3).copyTo(R_R21);
    m_R_T21.rowRange(0,3).col(3).copyTo(R_t21);

    cout << "T21 = \n" << m_R_T21 << endl << "R21 = \n" << R_R21 << endl << "t21 = \n" << R_t21 << endl;

//    for (auto lit = KF.begin(), lend = KF.end(); lit != lend; lit++) {
//        // 更新关键帧位姿
//        cout << "11 " << endl;
//        KeyFrame* pKF = *lit;
//        cv::Mat Tciw2 = pKF->GetPose() * m_R_T12;
//        pKF->SetPose(Tciw2);
//    }
    mp_R_LocalMapper->RequestStop();
    mp_R_LocalMapper->EmptyQueue();

    while(!mp_R_LocalMapper->isStopped())
    {
        usleep(1000);
    }

    {
        std::unique_lock<std::mutex> lock(Cur_Map->mMutexMapUpdate);

        // 更新关键帧位姿
        vector<KeyFrame*> vKFs = Cur_Map->GetAllKeyFrames();
        for (auto lit = vKFs.begin(), lend = vKFs.end(); lit != lend; lit++) {
            KeyFrame* pKF = *lit;
            cv::Mat Tciw2 = pKF->GetPose() * m_R_T12;
            pKF->SetPose(Tciw2);
//            pKF->UpdateConnections();
        }

        // 更新地图点
        vector<MapPoint*> vMPs = Cur_Map->GetAllMapPoints();
        for (auto sit = vMPs.begin(), send = vMPs.end(); sit != send; sit++) {
            MapPoint* pMP = *sit;
            pMP->SetWorldPos(R_R21 * pMP->GetWorldPos() + R_t21);
            pMP->UpdateNormalAndDepth();
        }

        // 更新地图线
        vector<MapLine*> vMLs = Cur_Map->GetAllMapLines();
        for (auto vit = vMLs.begin(), vend = vMLs.end(); vit != vend; vit++) {
            MapLine* pML = *vit;
            if (pML->isBad() || !pML) continue;
            Eigen::Vector3d SP = pML->GetWorldPos().head(3);
            Eigen::Vector3d EP = pML->GetWorldPos().tail(3);

            SP = Converter::toMatrix3d(R_R21) * SP + Converter::toVector3d(R_t21);
            EP = Converter::toMatrix3d(R_R21) * EP + Converter::toVector3d(R_t21);
            pML->SetWorldPos(SP,EP);
            pML->UpdateNormalAndDepth();
        }
    }

    mp_R_LocalMapper->Release();

}

void Relocalization::UpdatePose2(Map *Cur_Map) {
    cv::Mat R_R21, R_t21;
    m_R_T21.rowRange(0,3).colRange(0,3).copyTo(R_R21);
    m_R_T21.rowRange(0,3).col(3).copyTo(R_t21);

    cout << "T21 = \n" << m_R_T21 << endl << "R21 = \n" << R_R21 << endl << "t21 = \n" << R_t21 << endl;

    mp_R_LocalMapper->RequestStop();
    mp_R_LocalMapper->EmptyQueue();

    while(!mp_R_LocalMapper->isStopped())
    {
        usleep(1000);
    }

    {
        std::unique_lock<std::mutex> lock(Cur_Map->mMutexMapUpdate);
        // 更新关键帧位姿
        vector<KeyFrame*> vKFs = Cur_Map->GetAllKeyFrames();
        for (auto lit = vKFs.begin(), lend = vKFs.end(); lit != lend; lit++) {
            KeyFrame* pKFi = *lit;

            // 更新地图点
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for (size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++) {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                pMPi->SetWorldPos(R_R21 * pMPi->GetWorldPos() + R_t21);
                pMPi->UpdateNormalAndDepth();
            }
            // 更新地图线
            vector<MapLine*> vpMLsi = pKFi->GetMapLineMatches();
            for(size_t iML=0, endMLi = vpMLsi.size(); iML<endMLi; iML++) {
                MapLine* pMLi = vpMLsi[iML];
                if(!pMLi)
                    continue;
                if(pMLi->isBad())
                    continue;
                Eigen::Vector3d SP = pMLi->GetWorldPos().head(3);
                Eigen::Vector3d EP = pMLi->GetWorldPos().tail(3);

                SP = Converter::toMatrix3d(R_R21) * SP + Converter::toVector3d(R_t21);
                EP = Converter::toMatrix3d(R_R21) * EP + Converter::toVector3d(R_t21);
                pMLi->SetWorldPos(SP,EP);
                pMLi->UpdateNormalAndDepth();
            }

            cv::Mat Tciw2 = pKFi->GetPose() * m_R_T12;
            pKFi->SetPose(Tciw2);
            pKFi->UpdateConnections();
        }

    }
    mp_R_LocalMapper->Release();

}

void Relocalization::UpdatePose3(Map *Cur_Map) {
    cv::Mat R_R21, R_t21;
    m_R_T21.rowRange(0,3).colRange(0,3).copyTo(R_R21);
    m_R_T21.rowRange(0,3).col(3).copyTo(R_t21);

    while (!lKFs.empty()) {
        KeyFrame* pKFi = lKFs.front();
        lKFs.pop_front();

        cv::Mat Rciw2 = pKFi->GetImuRotation().t() * R_R21.t();
//    vector<float> q = Converter::toQuaternion(Rciw2);
        cv::Mat tw2b = R_R21 * pKFi->GetImuPosition() + R_t21;
        R_pose Pair_Pose = std::pair<cv::Mat, cv::Mat>(Rciw2, tw2b);
        mp_R_pose.insert(std::pair<double, R_pose>(pKFi->mTimeStamp, Pair_Pose));
    }
}

std::map<double, R_pose> Relocalization::GetAllPose() {
    lock_guard<std::mutex> lock(mMutexRKFS);
    return mp_R_pose;
}

void Relocalization::add(R_Frame* R_F) {
//        cout << "111" << endl;
        if (R_F->m_BowVector.empty()) {
            cout << "empty " << endl;
            return;
        }
//        cout << "size = " << R_F->m_BowVector.size() << endl;
//        m_InvertedFile_R.reserve(R_F->m_BowVector.size());
    for(DBoW2::BowVector::const_iterator vit= R_F->m_BowVector.begin(), vend=R_F->m_BowVector.end(); vit!=vend; vit++) {
//        cout << "push" << endl;
        m_InvertedFile_R[vit->first].push_back(R_F);
//        cout << "pop" << endl;
    }
//    cout << "222" << endl;
}

vector<R_Frame*> Relocalization::DetectRelocalization(KeyFrame *pKF) {
    vector<R_Frame*> vpRelocCandidates;
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++) {
        list<R_Frame*> &lRFs = m_InvertedFile_R[vit->first];

        for(auto lit=lRFs.begin(), lend= lRFs.end(); lit!=lend; lit++)
        {
            R_Frame* pRFi=*lit;
            vpRelocCandidates.push_back(pRFi);
        }
    }
    return vpRelocCandidates;
}

void Relocalization::RequestReset() {
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void Relocalization::RequestResetActiveMap(Map *pMap) {
    {
        unique_lock<mutex> lock(mMutexReset);
        cout << "LM: Active map reset recieved" << endl;
        mbResetRequestedActiveMap = true;
    }
    cout << "LM: Active map reset, waiting..." << endl;

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequestedActiveMap)
                break;
        }
        usleep(3000);
    }
    cout << "LM: Active map reset, Done!!!" << endl;
}

void Relocalization::ResetIfRequested() {
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        mlRelocalKeyFrames.clear();
        mbResetRequested=false;
    }
    else if(mbResetRequestedActiveMap)
    {
        mlRelocalKeyFrames.clear();
        mbResetRequestedActiveMap=false;

    }
}

bool Relocalization::ParseCamParamFile(cv::FileStorage &fSettings) {
    string img_path = fSettings["Relocalization.data"];
    string voc_path = fSettings["Relocalization.Path"];
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];
    if (img_path.empty() || voc_path.empty()) {
        std::cerr << "Vocabulary/Image dose not exist...." << std::endl;
        return false;
    }
    else {
        m_imgPath = img_path;
        m_vocPath = voc_path;
        orb_exetractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        return true;
    }
    return false;
}

void Relocalization::InsertKeyFrame(KeyFrame *pKF) {
    unique_lock<mutex> lock(mMutexRelocalQueue);
    mlRelocalKeyFrames.push_back(pKF);
}

void Relocalization::ClearKF() {
    unique_lock<mutex> lock(mMutexRelocalQueue);
    mlRelocalKeyFrames.clear();
}

bool Relocalization::CheckNewKeyFrames() {
    unique_lock<mutex> lock(mMutexRelocalQueue);
    return(!mlRelocalKeyFrames.empty());
}


//void Relocalization::SetTracker(Tracking *pTracker) {
//    mp_R_Tracker = pTracker;
//}

void Relocalization::SetLocalMapper(LocalMapping *pLocalMapper) {
    mp_R_LocalMapper = pLocalMapper;
}

}

