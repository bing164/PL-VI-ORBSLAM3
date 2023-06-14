//
// Created by bing on 2023/6/12.
//

#include "R_Optimizer.h"

namespace ORB_SLAM3 {

int R_Optimizer::R_PoseOptimization(R_Frame *Bow_F, KeyFrame *Cur_F) {
    g2o::SparseOptimizer optimizer;
    auto linearSolver = new g2o::LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
    BlockSolver_6_3 *solver_ptr = new BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // set Frame vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    Eigen::Matrix<double, 3, 1> t(0,0,0);
    vSE3->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), t));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    const int N = Bow_F->m_pts3d.size();
    cout << "N = " << N << endl;

    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdge;
    vpEdge.reserve(N);

    for (size_t i = 0; i < N; i++) {
        cv::Point3d p3d = Bow_F->m_pts3d[i];
        cv::Point2d p2d = Bow_F->m_pts2d[i];

        Eigen::Matrix<double, 2, 1> obs;
        obs << p2d.x, p2d.y;
        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());

        e->fx = Bow_F->m_K.at<double>(0,0);
        e->fy = Bow_F->m_K.at<double>(1,1);
        e->cx = Bow_F->m_K.at<double>(0,2);
        e->cy = Bow_F->m_K.at<double>(1,2);
        e->Xw << p3d.x, p3d.y, p3d.z;

        vpEdge.push_back(e);
        optimizer.addEdge(e);
    }

    const float chi2Mono[4]={5.991,5.991,5.991,5.991};

    int nBad = 0;
    std::vector<bool> isBad;
    isBad.reserve(N);
    for (size_t it = 0; it < 4; it++) {
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        nBad = 0;
        isBad.clear();
        for (size_t i = 0; i < vpEdge.size(); i++) {
            g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdge[i];
            e->computeError();
            float ch = e->chi2();
            if (ch > chi2Mono[it]) {
                e->setLevel(1);
                isBad.push_back(true);
                nBad++;
            } else {
                isBad.push_back(false);
                e->setLevel(0);
            }
            cout << "ch = " << ch << endl;
        }
    }

    g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3Quat_recov = vSE3_recov->estimate();
    std::cout << "Tcr = \n" << SE3Quat_recov << std::endl;

    Bow_F->SetTcr(Converter::toCvMat(SE3Quat_recov));


}
}

