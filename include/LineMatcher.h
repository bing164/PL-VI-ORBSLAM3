/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include "gridStructure.h"
#include "Frame.h"
#include "MapPoint.h"
#include "MapLine.h"

namespace ORB_SLAM3 {

class Frame;
class MapPoint;
class MapLine;

typedef std::pair<int, int> point_2d;
typedef std::pair<point_2d, point_2d> line_2d;

inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
    return (a.first*b.first + a.second*b.second);
}

inline void normalize(std::pair<double, double> &v) {
    double magnitude = std::sqrt(dot(v, v));

    v.first /= magnitude;
    v.second /= magnitude;
}

// 比较线特征距离的两种方式，自己添加的
    struct compare_descriptor_by_NN_dist
    {
        inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
            return ( a[0].distance < b[0].distance);
        }
    };

    struct conpare_descriptor_by_NN12_dist
    {
        inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
            return ((a[1].distance - a[0].distance) > (b[1].distance - b[0].distance));
        }
    };

// 按描述子之间距离的从小到大方式排序
    struct sort_descriptor_by_queryIdx
    {
        inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
            return ( a[0].queryIdx < b[0].queryIdx );
        }
    };

//    struct sort_lines_by_response
//    {
//        inline bool operator()(const KeyLine& a, const KeyLine& b){
//            return ( a.response > b.response );
//        }
//    };

class LineMatcher
{
public:
    static const int TH_HIGH, TH_LOW;
    int static matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int static match(const std::vector<MapLine*> &mvpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12);

    int static match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int SerachForInitialize(Frame &InitialFrame, Frame &CurrentFrame, std::vector<std::pair<int,int>> &LineMatches);

    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<std::pair<size_t, size_t>> &vMatchedPairs);

    int static distance(const cv::Mat &a, const cv::Mat &b);

    int static matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

    int static SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const GridStructure &grid, const float &th, const float &angth);

    int Fuse(KeyFrame* pKF, const vector<MapLine *> &vpMapLines, const float th=3.0);

    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
};

} // namesapce ORB_SLAM3
