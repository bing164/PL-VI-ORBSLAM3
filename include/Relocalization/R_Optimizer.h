//
// Created by bing on 2023/6/12.
//

#ifndef ORB_SLAM3_RELOCALIZATION_R_OPTIMIZER_H
#define ORB_SLAM3_RELOCALIZATION_R_OPTIMIZER_H

#include "R_Frame.h"
#include "../Thirdparty/g2o/g2o/core/block_solver.h"
#include "../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "../Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "../Thirdparty/g2o/g2o/core/block_solver.h"
#include "../Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "../Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "../Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "../Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include "eigen3/Eigen/Core"

namespace ORB_SLAM3 {
class R_Frame;
class R_Optimizer {
public:
    R_Optimizer() {}

    static int R_PoseOptimization(R_Frame* Bow_F, KeyFrame* Cur_F);

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> BlockSolver_6_3;
    typedef g2o::LinearSolverDense<BlockSolver_6_3::PoseMatrixType> LinearSolverType;
};
}
#endif //ORB_SLAM3_RELOCALIZATION_R_OPTIMIZER_H
