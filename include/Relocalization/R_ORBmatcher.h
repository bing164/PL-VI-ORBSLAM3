//
// Created by bing on 2023/6/12.
//

#ifndef ORB_SLAM3_RELOCALIZATION_R_ORBMATCHER_H
#define ORB_SLAM3_RELOCALIZATION_R_ORBMATCHER_H

#include "opencv2/opencv.hpp"
#include "R_Frame.h"
#include "ORBmatcher.h"

namespace ORB_SLAM3 {
class R_Frame;
class R_ORBmatcher{
public:
    R_ORBmatcher(float nnratio = 0.6, bool checkOri = true);

    int SearchForRelocalization(R_Frame &F1, KeyFrame *F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    int SearchForRelocalizationByOpenCV(R_Frame &F1, KeyFrame *F2);
protected:
    void R_ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

public:
    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:
    float m_nnratio;
    bool m_checkOri;
};


}


#endif //ORB_SLAM3_RELOCALIZATION_R_ORBMATCHER_H
