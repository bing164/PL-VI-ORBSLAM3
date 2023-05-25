/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "TwoViewReconstruction.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include<thread>


using namespace std;
namespace ORB_SLAM3
{

TwoViewReconstruction::TwoViewReconstruction(cv::Mat& K, float sigma, int iterations)
{
    mK = K.clone();

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const vector<int> &vMatches12,
                                        cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    mvKeys1.clear();
    mvKeys2.clear();

    mvKeys1 = vKeys1;
    mvKeys2 = vKeys2;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

    DUtils::Random::SeedRandOnce(0);

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if(SH+SF == 0.f) return false;
    float RH = SH/(SH+SF);

    float minParallax = 1.0;

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if(RH>0.50) // if(RH>0.40)
    {
        //cout << "Initialization from Homography" << endl;
        return ReconstructH(vbMatchesInliersH,H, mK,R21,t21,vP3D,vbTriangulated,minParallax,50);
    }
    else //if(pF_HF>0.6)
    {
        //cout << "Initialization from Fundamental" << endl;
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,minParallax,50);
    }
}

bool TwoViewReconstruction::ReconstructwithLine(const Frame &InitialFrame, const Frame &CurrentFrame,
                                                const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                                                vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,
                                                vector<std::pair<int, int>> &vLineMatches,
                                                vector<cv::Point3f> &vLineS3D, vector<cv::Point3f> &vLineE3D,
                                                vector<bool> &vbLineTriangulated) {
    mvKeys1.clear();
    mvKeys2.clear();

    mvKeys1 = InitialFrame.mvKeysUn;
    mvKeys2 = CurrentFrame.mvKeysUn;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
        if(vMatches12[i]>=0) {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i] = true;
        } else {
            mvbMatched1[i] = false;
        }
    }

    const int N = mvMatches12.size();
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for (int i = 0; i < N; i++) {
        vAllIndices.push_back(i);
    }

    mvSets = vector<vector<size_t>>(mMaxIterations,vector<size_t>(8,0));
    DUtils::Random::SeedRandOnce(0);
    for (int it = 0; it < mMaxIterations; it++) {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for (size_t j = 0; j < 8; j++) {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size() - 1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if(SH+SF == 0.f) return false;
    float RH = SH/(SH+SF);

    float minParallax = 1.0;

    mvKeyLines1.clear();
    mvKeyLines2.clear();
    mvKeyLines1 = InitialFrame.mvKeysUn_Line;
    mvKeyLines2 = CurrentFrame.mvKeysUn_Line;
    mvKeyLineFunctions1 = InitialFrame.mvKeyLineFunctions;
    mvKeyLineFunctions2 = CurrentFrame.mvKeyLineFunctions;
    mvbLineMatched1.clear();
    mvLineMatches12.clear();
    mvLineMatches12.reserve(mvKeyLines2.size());
    mvbLineMatched1.resize(mvKeyLines1.size());

    vector<Match> lineMatches;
    for (size_t i = 0, iend = vLineMatches.size(); i < iend; i++) {
        if(vLineMatches[i].first >=0 && vLineMatches[i].second >= 0) {
            Match lmatch;
            lmatch.first = vLineMatches[i].first;
            lmatch.second = vLineMatches[i].second;
            lineMatches.push_back(lmatch);
        }
    }

    if(RH>0.50)
    {
        bool isReconstructH;
        isReconstructH = ReconstructHwithLine(vbMatchesInliersH,lineMatches,H,mK,R21,t21,vP3D,vLineS3D, vLineE3D,vbTriangulated,vbLineTriangulated,1.0,50);
        return isReconstructH;
    } else{
        bool isReconstructF;
        isReconstructF = ReconstructFwithLine(vbMatchesInliersF,lineMatches,F,mK,R21,t21,vP3D,vLineS3D, vLineE3D,vbTriangulated,vbLineTriangulated,1.0,50);
        return isReconstructF;
    }
}

void TwoViewReconstruction::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


void TwoViewReconstruction::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


cv::Mat TwoViewReconstruction::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(2*N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

cv::Mat TwoViewReconstruction::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    return  u*cv::Mat::diag(w)*vt;
}

float TwoViewReconstruction::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

float TwoViewReconstruction::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool TwoViewReconstruction::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool TwoViewReconstruction::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

bool TwoViewReconstruction::ReconstructHwithLine(std::vector<bool> &vbMatchesInliers, vector<Match> &vLineMatchesH,
                                                 cv::Mat &H21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                                                 std::vector<cv::Point3f> &vP3D, vector<cv::Point3f> &vLineS3D,
                                                 vector<cv::Point3f> &vLineE3D, std::vector<bool> &vbTriangulated,
                                                 vector<bool> &vinliers, float minParallax, int minTriangulated) {
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        ReconstructLine(vLineMatchesH, mK, R21, t21, mvKeyLineFunctions1, mvKeyLineFunctions2, vLineS3D, vLineE3D, vinliers);    //三角化建立线特征的3D端点
        return true;
    }

    return false;
}

bool TwoViewReconstruction::ReconstructFwithLine(vector<bool> &vbMatchesInliers, vector<Match> &vLineMatchesF,
                                                 cv::Mat &F21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                                                 vector<cv::Point3f> &vP3D, vector<Point3f> &vLineS3D,
                                                 vector<Point3f> &vLineE3D, vector<bool> &vbTriangulated,
                                                 vector<bool> &vinliers, float minParallax, int minTriangulated) {
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            ReconstructLine(vLineMatchesF, mK, R21, t21, mvKeyLineFunctions1, mvKeyLineFunctions2, vLineS3D, vLineE3D, vinliers);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            ReconstructLine(vLineMatchesF, mK, R21, t21, mvKeyLineFunctions1, mvKeyLineFunctions2, vLineS3D, vLineE3D, vinliers);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            ReconstructLine(vLineMatchesF, mK, R21, t21, mvKeyLineFunctions1, mvKeyLineFunctions2, vLineS3D, vLineE3D, vinliers);

            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            ReconstructLine(vLineMatchesF, mK, R21, t21, mvKeyLineFunctions1, mvKeyLineFunctions2, vLineS3D, vLineE3D, vinliers);

            return true;
        }
    }

    return false;
}

void TwoViewReconstruction::ReconstructLine(vector<Match> &vLineMatches, cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                                            vector<Vector3d> &vKeyLineFunctions1, vector<Vector3d> &vKeyLineFunctions2,
                                            std::vector<cv::Point3f> &vLineS3D, std::vector<cv::Point3f> &vLineE3D,
                                            std::vector<bool> &vbLineTriangulated) {
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vLineS3D.resize(vLineMatches.size(),Point3f(0,0,0));
    vLineE3D.resize(vLineMatches.size(),Point3f(0,0,0));
    vbLineTriangulated.resize(vLineMatches.size(),true);

    // 得到第一个相机的投影矩阵，并以第一个相机的光心作为世界坐标系
    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // 得到第二个相机的投影矩阵，并以第二个相机的光心作为世界坐标系
    cv::Mat P2(3, 4, CV_32F);
    R21.copyTo(P2.rowRange(0,3).colRange(0,3));
    t21.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;
    cv::Mat O2 = -R21.t()*t21;

    vector<double> errorC1, errorC2;
    errorC1.resize(vLineMatches.size(), 0);
    errorC2.resize(vLineMatches.size(), 0);

    for(size_t i=0, iend = vLineMatches.size(); i<iend; i++) {
        // 匹配的两条特征线
        const KeyLine &kl1 = mvKeyLines1[vLineMatches[i].first];
        const KeyLine &kl2 = mvKeyLines2[vLineMatches[i].second];
        cv::Mat L3dSC1; //线特征的3D起始点在camera1下的坐标
        cv::Mat L3dEC1; //线特征的3D终止点在camera1下的坐标
        Vector3d &klf1 = mvKeyLineFunctions1[vLineMatches[i].first];    //第一帧的特征线段所在直线系数
        Vector3d &klf2 = mvKeyLineFunctions2[vLineMatches[i].second];   //第二帧的特征线段所在直线系数

        LineTriangulate(kl1, kl2, P1, P2, klf1, klf2, L3dSC1, L3dEC1);

        if(!isfinite(L3dSC1.at<float>(0)) || !isfinite(L3dSC1.at<float>(1)) || !isfinite(L3dSC1.at<float>(2)) ||
           !isfinite(L3dEC1.at<float>(0)) || !isfinite(L3dEC1.at<float>(1)) || !isfinite(L3dEC1.at<float>(2)) )
        {
            continue;
        }

        vLineS3D[i] = cv::Point3f(L3dSC1.at<float>(0), L3dSC1.at<float>(1), L3dSC1.at<float>(2));
        vLineE3D[i] = cv::Point3f(L3dEC1.at<float>(0), L3dEC1.at<float>(1), L3dEC1.at<float>(2));

        // 线特征的两个端点在camera2下的坐标
        cv::Mat L3dSC2 = R21*L3dSC1 + t21;
        cv::Mat L3dEC2 = R21*L3dEC1 + t21;

        //***********根据线特征的重投影误差****************
        // step1: 计算起始点和终止点在第一帧上的重投影误差
        // 1.1：计算线段所在的直线方程
        Vector3d l_obs1 = vKeyLineFunctions1[vLineMatches[i].first];

        // 1.2:计算起始点和终止点的3D坐标反投影至图像平面上的坐标
        float im1Startx, im1Starty;
        float invZ1start = 1.0/L3dSC1.at<float>(2);
        im1Startx = fx*L3dSC1.at<float>(0)*invZ1start+cx;
        im1Starty = fy*L3dSC1.at<float>(1)*invZ1start+cy;

        float im1Endx, im1Endy;
        float invZ1end = 1.0/L3dEC1.at<float>(2);
        im1Endx = fx*L3dEC1.at<float>(0)*invZ1end+cx;
        im1Endy = fx*L3dEC1.at<float>(1)*invZ1end+cy;

        // 1.3：计算起始点和终止点的2D反投影坐标到直线的距离，误差为距离之和
        Vector2d err_l1;
        err_l1(0) = l_obs1(0)*im1Startx + l_obs1(1)*im1Starty + l_obs1(2);
        err_l1(1) = l_obs1(0)*im1Endx + l_obs1(1)*im1Endy + l_obs1(2);
        double err1 = err_l1.norm();
        errorC1[i] = err1;

        // step2：计算起始点和终止点在第二帧上的重投影误差
        // 2.1：计算第二帧的线段所在直线的方程
        Vector3d l_obs2 = vKeyLineFunctions2[vLineMatches[i].second];

        // 2.2：计算起始点和终止点的3D坐标反投影至图像平面上的坐标
        float im2Startx, im2Starty;
        float invZ2start = 1.0/L3dSC2.at<float>(2);
        im2Startx = fx*L3dSC2.at<float>(0)*invZ2start+cx;
        im2Starty = fy*L3dSC2.at<float>(1)*invZ2start+cy;

        float im2Endx, im2Endy;
        float invZ2end = 1.0/L3dEC2.at<float>(2);
        im2Endx = fx*L3dEC2.at<float>(0)*invZ2end+cx;
        im2Endy = fy*L3dEC2.at<float>(1)*invZ2end+cy;

        // 2.3：计算起始点和终止点的2D反投影坐标到直线的距离，误差为距离之和
        Vector2d err_l2;
        err_l2(0) = l_obs2(0)*im2Startx + l_obs2(1)*im2Starty + l_obs2(2);
        err_l2(1) = l_obs2(0)*im2Endx + l_obs2(1)*im2Endy + l_obs2(2);
        double err2 = err_l2.norm();
        errorC2[i] = err2;

    }

    //**********去除线特征集中的异常值**********
    double inlier_th_l1 = 2.0 * vector_mad(errorC1);
    double inlier_th_l2 = 2.0 * vector_mad(errorC2);
    for(int i=0; i<vLineS3D.size(); i++)
    {
        if(errorC1[i]>inlier_th_l1 || errorC2[i]>inlier_th_l2)
        {
            vbLineTriangulated[i] = false;
        }
    }
}

void TwoViewReconstruction::LineTriangulate(const KeyLine &kl1, const KeyLine &kl2, const cv::Mat &P1,
                                            const cv::Mat &P2, const Vector3d &klf1, const Vector3d &klf2,
                                            cv::Mat &LineStart3D, cv::Mat &LineEnd3D) {
    const float &fx1 = P1.at<float>(0,0);
    const float &fy1 = P1.at<float>(1,1);
    const float &cx1 = P1.at<float>(0,2);
    const float &cy1 = P1.at<float>(1,2);
    const float &invfx1 = 1.0/P1.at<float>(0,0);
    const float &invfy1 = 1.0/P1.at<float>(1,1);
    cv::Mat K=cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = P1.at<float>(0,0);
    K.at<float>(1,1) = P1.at<float>(1,1);
    K.at<float>(0,2)= P1.at<float>(0,2);
    K.at<float>(1,2) = P1.at<float>(1,2);


    Mat lineF1 = (Mat_<float>(3,1) << klf1(0), klf1(1), klf1(2));
    Mat lineF2 = (Mat_<float>(3,1) << klf2(0), klf2(1), klf2(2));

    cv::Mat e = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Tw(3,4,CV_32F);
    e.rowRange(0,3).colRange(0,3).copyTo(Tw.colRange(0,3));
    e.rowRange(0,3).col(3).copyTo(Tw.col(3));
    cv::Mat p1=K*Tw;
    cv::Mat StartC1, EndC1;
    StartC1 = (cv::Mat_<float>(3,1) << (kl1.startPointX-cx1)*invfx1, (kl1.startPointY-cy1)*invfy1, 1.0);
    EndC1 = (cv::Mat_<float>(3,1) << (kl1.endPointX-cx1)*invfx1, (kl1.endPointY-cy1)*invfy1, 1.0);

    cv::Mat A(4, 4, CV_32F);
    A.row(0) = lineF1.t()*p1;
    A.row(1) = lineF2.t()*P2;
    A.row(2) = StartC1.at<float>(0) * Tw.row(2) - Tw.row(0);
    A.row(3) = StartC1.at<float>(1) * Tw.row(2) - Tw.row(1);

    cv::Mat u1, w1, vt1;
    cv::SVD::compute(A, w1, u1, vt1, cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    LineStart3D = vt1.row(3).t();

    LineStart3D = LineStart3D.rowRange(0,3)/LineStart3D.at<float>(3);
//    cout<<LineStart3D<<endl;
    // 终止点
    cv::Mat B(4, 4, CV_32F);
    B.row(0) = lineF1.t()*p1;
    B.row(1) = lineF2.t()*P2;
    B.row(2) = EndC1.at<float>(0) *Tw.row(2) -Tw.row(0);
    B.row(3) = EndC1.at<float>(1) * Tw.row(2) - Tw.row(1);

    cv::Mat u2, w2, vt2;
    cv::SVD::compute(B, w2, u2, vt2, cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    LineEnd3D = vt2.row(3).t();
    LineEnd3D = LineEnd3D.rowRange(0,3)/LineEnd3D.at<float>(3);
//    cout << "start = " << LineStart3D.t() << " end = " << LineEnd3D.t() << endl;
}

void TwoViewReconstruction::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}


int TwoViewReconstruction::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void TwoViewReconstruction::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

} //namespace ORB_SLAM
