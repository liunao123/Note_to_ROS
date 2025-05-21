// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)

#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include "ceres/ceres.h"
#include "read_g2o.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "pose_graph_3d_error_term.h"
#include "types.h"

DEFINE_string(input, "", "The pose graph definition filename in g2o format.");

namespace ceres {
namespace examples {
namespace {

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                              MapOfPoses* poses,
                              ceres::Problem* problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction* loss_function = nullptr;
  ceres::Manifold* quaternion_manifold = new EigenQuaternionManifold;

  for (const auto& constraint : constraints) {
    auto pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    auto pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);
    
    problem->AddResidualBlock(cost_function,
                              loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
                         quaternion_manifold);
    problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
                         quaternion_manifold);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  auto pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != nullptr);

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  // options.linear_solver_type = ceres::SPARSE_SCHUR ;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const MapOfPoses& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (const auto& pair : poses) {
    outfile << std::setprecision(3) << pair.first << " " << pair.second.p.x() << " " << pair.second.p.y() << " " << pair.second.p.z() << " "
            << std::setprecision(6) << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}



}  // namespace
}  // namespace examples
}  // namespace ceres


bool WriteG2oFile(const std::string& filename,
                  ceres::examples::MapOfPoses poses,
                  ceres::examples::VectorOfConstraints constraints) {

  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (const auto& pair : poses) {
    outfile << pair.second.name() << " " << pair.first << " " 
            << std::setiosflags( std::ios::fixed) << std::setprecision(5) << pair.second.p.x()  << " " << pair.second.p.y() << " " << pair.second.p.z() << " "
            << std::setprecision(6) << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  outfile.close();

  outfile.open(filename.c_str(), std::istream::app);
  for (auto& constraint : constraints) {
    ceres::examples::Pose3d & t_be = constraint.t_be;
    outfile << constraint.name()  << " " << constraint.id_begin << " " << constraint.id_end << " "
            << std::setiosflags( std::ios::fixed) << std::setprecision(5) << t_be.p.x() << " " << t_be.p.y() << " " << t_be.p.z() << " "
            << std::setiosflags( std::ios::fixed) << std::setprecision(6) << t_be.q.x() << " " << t_be.q.y() << " "
            << t_be.q.z() << " " << t_be.q.w() << " "
            << std::setiosflags( std::ios::fixed) << std::setprecision(0) ;
    for (int i = 0; i < 6 ; ++i)
    {
      for (int j = i; j < 6 ; ++j)
      {
        outfile << constraint.information(i, j) << " ";
      }
    }
    outfile << '\n';
  }
  outfile.close();
  std::cout << "Write g2o file done : " << filename << std::endl;

  return true;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  ceres::examples::MapOfPoses poses;
  ceres::examples::VectorOfConstraints constraints;
  // std::string g2o_file = "/opt/csg/slam/navs/temp/slam_ceres/sphere.g2o";
  // std::string g2o_file = "/opt/csg/slam/navs/pose_graph.g2o";
  std::string g2o_file = "../sphereo.g2o";

  if ( argc == 2 )
  {
    g2o_file = std::string(argv[1]);
  }
  std::cout << "g2o_file: " << g2o_file << '\n';
  

  // 边是由位姿计算出来的 优化不会进行
  // std::string g2o_file = "/home/map/region5-10/pose_graph/graph.g2o";

  CHECK(ceres::examples::ReadG2oFile(g2o_file, &poses, &constraints))
      << "Error reading the file: " << FLAGS_input;

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  CHECK(ceres::examples::OutputPoses("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";

  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);

  CHECK(ceres::examples::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
      << "Error outputting to poses_original.txt";

  // // constraints
  // std::cout << "Number of poses: " << poses.size() << '\n';
  // std::cout << "Number of constraints: " << constraints.size() << '\n';

  WriteG2oFile("poses_optimized.g2o", poses, constraints);
  system(  std::string("evo_traj tum poses_optimized.txt poses_original.txt -p --plot_mode xy").c_str() );
  std::cout << "evo_traj tum poses_optimized.txt poses_original.txt -p --plot_mode xy " << poses.size() << '\n';

  return 0;

}
