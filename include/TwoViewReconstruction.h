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

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

#include<opencv2/opencv.hpp>
#include "line_descriptor_custom.hpp"
#include "line_descriptor/descriptor_custom.hpp"
#include "opencv2/line_descriptor/descriptor.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <complex>
#include "Frame.h"

#include <unordered_set>
using namespace std;
using namespace cv;
using namespace line_descriptor;
using namespace Eigen;

namespace ORB_SLAM3
{

class Frame;
class TwoViewReconstruction
{
    typedef std::pair<int,int> Match;

public:

    // Fix the reference frame
    TwoViewReconstruction(cv::Mat& k, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    bool Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated);

    bool ReconstructwithLine(const Frame &InitialFrame, const Frame &CurrentFrame,
                             const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,
                             vector<bool> &vbTriangulated,vector<std::pair<int, int>> &vLineMatches, vector<cv::Point3f> &vLineS3D,
                             vector<cv::Point3f> &vLineE3D, vector<bool> &vbLineTriangulated);

private:

    void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    void FindFundamental(std::vector<bool> &vbInliers, float &score, cv::Mat &F21);

    cv::Mat ComputeH21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);
    cv::Mat ComputeF21(const std::vector<cv::Point2f> &vP1, const std::vector<cv::Point2f> &vP2);

    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, std::vector<bool> &vbMatchesInliers, float sigma);

    float CheckFundamental(const cv::Mat &F21, std::vector<bool> &vbMatchesInliers, float sigma);

    bool ReconstructF(std::vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructH(std::vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D,std:: vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructHwithLine(std::vector<bool> &vbMatchesInliers, vector<Match> &vLineMatchesH,cv::Mat &H21, cv::Mat &K,
                              cv::Mat &R21, cv::Mat &t21,std::vector<cv::Point3f> &vP3D,vector<cv::Point3f> &vLineS3D, vector<cv::Point3f> &vLineE3D,
                              std:: vector<bool> &vbTriangulated,vector<bool> &vinliers, float minParallax, int minTriangulated);

    bool ReconstructFwithLine(vector<bool> &vbMatchesInliers, vector<Match> &vLineMatchesF, cv::Mat &F21, cv::Mat &K,
                              cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,vector<Point3f> &vLineS3D, vector<Point3f> &vLineE3D,
                              vector<bool> &vbTriangulated, vector<bool> &vinliers, float minParallax, int minTriangulated);


    //         用计算好的R t，来三角化计算线特征
    void ReconstructLine(vector<Match> &vLineMatchesH, cv::Mat &K,cv::Mat &R21, cv::Mat &t21,
                         vector<Vector3d> &vKeyLineFunctions1, vector<Vector3d> &vKeyLineFunctions2,
                         std::vector<cv::Point3f> &vLineS3D,std::vector<cv::Point3f> &vLineE3D,std::vector<bool> &vinliers);

    // 线特征的三角化
    // 结合极平面
    void LineTriangulate(const KeyLine &kl1, const KeyLine &kl2,const cv::Mat &P1, const cv::Mat &P2, const Vector3d &klf1, const Vector3d &klf2,
                         cv::Mat &LineStart3D, cv::Mat &LineEnd3D);

    /**
        * @brief 求一个vector数组的中位数绝对偏差MAD
        * 中位数绝对偏差MAD——median absolute deviation, 是单变量数据集中样本差异性的稳健度量。
        * MAD是一个健壮的统计量，对于数据集中异常值的处理比标准差更具有弹性，可以大大减少异常值对于数据集的影响
        * 对于单变量数据集 X={X1,X2,X3,...,Xn}, MAD的计算公式为：MAD(X)=median(|Xi-median(X)|)
        * @param residues
        * @return
        */
    inline double vector_mad(vector<double> residues)
    {
        if(residues.size()!=0)
        {
            // Return the standard deviation of vector with MAD estimation
            int n_samples = residues.size();
            sort(residues.begin(), residues.end());
            double median = residues[n_samples/2];
            for(int i=0; i<n_samples; i++)
                residues[i] = fabs(residues[i]-median);
            sort(residues.begin(), residues.end());
            double MAD = residues[n_samples/2];
            return 1.4826*MAD;
        } else
            return 0.0;
    }


    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void Normalize(const std::vector<cv::KeyPoint> &vKeys, std::vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);


    int CheckRT(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
                       const std::vector<Match> &vMatches12, std::vector<bool> &vbInliers,
                       const cv::Mat &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float &parallax);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    std::vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    std::vector<cv::KeyPoint> mvKeys2;

    // lines from Reference Frame (Frame 1)
    std::vector<cv::line_descriptor::KeyLine> mvKeyLines1;
    std::vector<Eigen::Vector3d> mvKeyLineFunctions1;
    std::vector<Match> mvLineMatches12;
    std::vector<bool> mvbLineMatched1;  // 记录reference frame的每个线特征在Current frame中是否有匹配对

    // lines from Current Frame (Frame 2)
    std::vector<cv::line_descriptor::KeyLine> mvKeyLines2;
    vector<Eigen::Vector3d> mvKeyLineFunctions2;



    // Current Matches from Reference to Current
    std::vector<Match> mvMatches12;
    std::vector<bool> mvbMatched1;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // Ransac max iterations
    int mMaxIterations;

    // Ransac sets
    std::vector<std::vector<size_t> > mvSets;

};

} //namespace ORB_SLAM

#endif // TwoViewReconstruction_H
