//
// Created by bing on 2023/6/7.
//

#include "Relocalization.h"

namespace ORB_SLAM3 {
//    std::vector<std::shared_ptr<R_Frame>> Vec_BOWFrame;
//    list<R_Frame> list_BowF;
//    std::vector<R_Frame> vec_BowF;
    cv::Mat K = (cv::Mat_<double>(3,3) << 518.0, 0, 325.5, 0, 519.0, 253.5, 0, 0, 1);
    typedef pair<double, int> PDI;
    struct cmp2 {
        bool operator()(const PDI& a, const PDI& b) {
            return a.first > b.first;
        }
    };

//    inline cv::Point2d pixel2cam(const cv::Point& p, const cv::Mat &K) {
//        return cv::Point2d(
//                (p.x - K.at<double>(0,2)) / K.at<double>(0,0),
//                (p.y - K.at<double>(1,2)) / K.at<double>(1,1)
//        );
//    }

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
            vec_Bow.push_back(currentFrame);
            add(&currentFrame);
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
    while (1) {
        if (CheckNewKeyFrames()) {
            KeyFrame* CurKF;
            cout << "Relocalization KFs = " << mlRelocalKeyFrames.size() << endl;
            {
                unique_lock<mutex> lock(mMutexNewKFs);
                CurKF = mlRelocalKeyFrames.front();
                mlRelocalKeyFrames.pop_front();
            }

            priority_queue<PDI, vector<PDI>, cmp2> que;
            R_ORBmatcher matcher(0.9, true);
            int k = 0;
            for (auto f : m_Vec_BowF) {
                double score = m_vocab->score(CurKF->mBowVec, f.m_BowVector);
                cout << "score = " << score << " id = " << f.FrameId << endl;

                que.push(PDI(score, k));
                if (que.size() > 3) {
                    que.pop();
                }
                k++;
            }

            int bestId = -1;
            int best_matches = 0;
            while (!que.empty()) {
                cout << "que111 = " << que.top().first << " 222 = " << que.top().second << endl;
                int nmatches = matcher.SearchForRelocalizationByOpenCV(m_Vec_BowF[que.top().second], CurKF);
//                cout << "nmatches = " << nmatches << endl;
                if (nmatches > best_matches) {
                    best_matches = nmatches;
                    bestId = que.top().second;
                }
                que.pop();
            }
            cout << "bestId = " << bestId << " best matches = " << best_matches << endl;
            R_Frame Bow_F = m_Vec_BowF[bestId];
            R_Frame::GetMapPoints(CurKF, &Bow_F);

//            R_Optimizer::R_PoseOptimization(&Bow_F, CurKF);


//            cv::Mat pose = Converter::toCvMat(m_Vec_BowF[k].m_pose.matrix());
//
//            m_CurKF->SetPose(pose);
//            m_CurKF->ComputeBoW();
//            std::vector<R_Frame*> RFs = DetectRelocalization(m_CurKF);
//            cout << "RFs = " << m_CurKF->mBowVec << endl;
//            for (auto i : RFs)
//                cout << "ID = " << i->FrameId << endl;

        }
        usleep(3000);
    }
}

void Relocalization::add(R_Frame* R_F) {
        cout << "111" << endl;
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

}

