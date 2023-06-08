//
// Created by bing on 2023/6/7.
//

#include "Relocalization.h"

namespace ORB_SLAM3 {
    std::vector<std::shared_ptr<R_Frame>> Vec_BOWFrame;
    cv::Mat K = (cv::Mat_<double>(3,3) << 518.0, 0, 325.5, 0, 519.0, 253.5, 0, 0, 1);

    void LoadImages(const std::string &strimagePath, ORBVocabulary* vocab) {
        std::string pose = strimagePath + "pose.txt";
        std::ifstream fin(pose);
        if (!fin) {
            std::cerr << "cannot find pose file!!!!" << std::endl;
            return;
        }
        std::string line;
        int i = 0;
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

//        Frame currentFrame = Frame(colorImgs, depthImgs, T, i+1, K);
            std::shared_ptr<R_Frame> currentFrame = std::make_shared<R_Frame>(colorImgs, depthImgs, T, i+1, to_string(clock()), K);
            Vec_BOWFrame.push_back(currentFrame);
//            DBoW2::BowVector Bow_vector;
            vocab->transform(currentFrame->m_des, currentFrame->m_BowVector, currentFrame->m_FeatVec, 4);
            cout << "end vocab" << endl;
            i++;
        }

    }


Relocalization::Relocalization(const std::string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    bool succed = ParseCamParamFile(fSettings);
    if (succed) {
        std::cout << "Yaml read finished...." << endl;
        m_vocab = new ORBVocabulary();
////        ORBVocabulary * vocab = new ORBVocabulary();
        m_vocab->loadFromTextFile(m_vocPath);
        LoadImages(m_imgPath, m_vocab);
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
            cout << "Relocalization KFs = " << mlRelocalKeyFrames.size() << endl;

        }
        usleep(3000);
    }
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
    if (img_path.empty() || voc_path.empty()) {
        std::cerr << "Vocabulary/Image dose not exist...." << std::endl;
        return false;
    }
    else {
        m_imgPath = img_path;
        m_vocPath = voc_path;
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

