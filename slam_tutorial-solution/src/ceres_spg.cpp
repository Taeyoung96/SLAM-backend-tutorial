/** Simple sparse pose graph example
 * 
 * 
 */

#include <ros/ros.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <map>
#include <fstream>

typedef Eigen::Quaterniond Quaternion;
typedef Eigen::Vector3d    Vec3;
typedef Eigen::Vector2d    Vec2;

// 3D pose data structure
struct Pose3D {
  Quaternion rotation;
  Vec3 translation;

  Pose3D(const Quaternion q, const Vec3 t){
    rotation = q.normalized();
    translation = t;
  }
};

// relative pose error struct
struct RelativePoseError3D {
  RelativePoseError3D(const Quaternion Rij,
                      const Vec3 tij) : Rij(Rij), tij(tij) {}
  template <typename T>
  bool operator()(const T* const rotation_i,
                  const T* const translation_i,
                  const T* const rotation_j,
                  const T* const translation_j,
                  T* residuals) const {

    Eigen::Quaternion<T> invRi( rotation_i[0], -rotation_i[1], 
                               -rotation_i[2], -rotation_i[3]);

    Eigen::Matrix<T,3,1> invti = invRi * 
                                 Eigen::Matrix<T,3,1>(-translation_i[0],
                                                      -translation_i[1],
                                                      -translation_i[2]);

    Eigen::Quaternion<T> Rj(rotation_j[0], rotation_j[1], 
                            rotation_j[2], rotation_j[3]);

    Eigen::Matrix<T,3,1> tj(translation_j[0],
                            translation_j[1],
                            translation_j[2]);

    Eigen::Quaternion<T> Rji = invRi * Rj;
    Eigen::Matrix<T,3,1> tji = invRi * tj + invti;

    Eigen::Quaternion<T> dR = Rji * Rij.cast<T>();
    Eigen::Matrix<T,3,1> dt = Rji * tij.cast<T>() + tji;

    // residuals.
    residuals[0] = dR.x();
    residuals[1] = dR.y();
    residuals[2] = dR.z();
    residuals[3] = dt(0);
    residuals[4] = dt(1);
    residuals[5] = dt(2);

    return true;
  }

  static ceres::CostFunction* Create(const Quaternion Rij,
                                     const Vec3 tij) {
    return (new ceres::AutoDiffCostFunction<RelativePoseError3D, 6, 4, 3, 4, 3>(
               new RelativePoseError3D(Rij, tij)));
  }

  const Quaternion Rij;
  const Vec3 tij;
};

std::vector<std::vector<double>> readCSV(const std::string fname) {
  std::ifstream pose_fid(fname.c_str());
  std::string line;

  std::vector<std::vector<double>> data;
  while(std::getline(pose_fid, line)) {
      std::vector<double> row;
      std::stringstream iss(line);
      std::string val;

      while (std::getline(iss, val, ',')) {
          double v = std::atof(val.c_str());
          row.push_back(v);
      }
      data.push_back(row);
  }

  return data;
}

void run() {
  // load initial guess of all the node poses
  const auto pose_data = readCSV(
      "/root/slam_ws/src/slam_tutorial/data/initial_poses.csv");
  std::map<size_t, Pose3D> initial_poses;
  for(const auto& row : pose_data) {
    size_t     pose_id(row[0]);
    Quaternion q(row[1], row[2], row[3], row[4]);
    Vec3       t(row[5], row[6], row[7]);
    Pose3D     pose(q, t);
    initial_poses.insert(std::make_pair(pose_id, pose));
  }
  std::cout << "Read " << initial_poses.size() << " poses." << std::endl;
  
  // load the relative pose constraints between nodes
  const auto constraint_data = readCSV(
      "/root/slam_ws/src/slam_tutorial/data/constraints.csv");
  std::map<std::pair<size_t, size_t>, Pose3D> constraints;
  for(const auto& row : constraint_data) {
    std::pair<size_t, size_t> pose_pair = std::make_pair(row[0], row[1]);
    Quaternion q(row[2], row[3], row[4], row[5]);
    Vec3       t(row[6], row[7], row[8]);
    Pose3D     pose(q, t);
    constraints.insert(std::make_pair(pose_pair, pose));
  }
  std::cout << "Read " << constraints.size() << " constraints." << std::endl;
  
  // we choose the node with id 0 to be fixed, so the all the result nodes
  // are optimized relative to this node
  size_t fixed_node_id = 0;

  // initialize the values of rotations and positions
  std::map<size_t, std::vector<double>> ceres_rotations;
  std::map<size_t, std::vector<double>> ceres_positions;
  for(const auto &pose : initial_poses) {
    size_t     id = pose.first;
    Quaternion q  = pose.second.rotation;
    Vec3       t  = pose.second.translation;

    std::vector<double> ceres_rotation = {q.w(), q.x(), q.y(), q.z()};
    std::vector<double> ceres_position = {t(0),t(1),t(2)};
    ceres_rotations[id] = ceres_rotation;
    ceres_positions[id] = ceres_position;
  }

  // declare the ceres problem 
  ceres::Problem problem;

  // the quaternion parameterization is used to let ceres solver know that
  // a parameter block should be solve as a quaternion, not 4 free values
  ceres::LocalParameterization *quaternion_parameterization =
      new ceres::QuaternionParameterization;

  // add all constraints to the defined problem
  for(const auto &constr : constraints) {
    // get the node id of the current constraint
    const size_t I = constr.first.first;
    const size_t J = constr.first.second;

    // loss function is also called robust loss function. it is a 
    // technique to reduce the effects of outliers. here we choose 
    // to use the Huber loss. 
    ceres::LossFunction* loss_func(new ceres::HuberLoss(0.1));
    
    // initialize the relative pose error function
    ceres::CostFunction* cost_function =
        RelativePoseError3D::Create(
            constr.second.rotation,
            constr.second.translation);

    // add the error term to the problem
    problem.AddResidualBlock(
        cost_function,
        loss_func,
        &(ceres_rotations[I][0]),
        &(ceres_positions[I][0]),
        &(ceres_rotations[J][0]),
        &(ceres_positions[J][0]));

    // let the ceres solver know that the rotation blocks are quaternions
    problem.SetParameterization(
        &(ceres_rotations[I][0]), 
        quaternion_parameterization);
    problem.SetParameterization(
        &(ceres_rotations[J][0]), 
        quaternion_parameterization);
  }

  // we want to set one of the nodes to be fixed during the optimization,
  // otherwise, the whole graph will become 'floating'. 
  problem.SetParameterBlockConstant(&(ceres_rotations[fixed_node_id][0]));
  problem.SetParameterBlockConstant(&(ceres_positions[fixed_node_id][0]));

  // define solver options (solver type, logging options etc.)
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true; // enable this to see progress

  // call the solve function to optimize the constructed problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // print out the optimization logging info
  // use summary.FullReport() to see the details
  std::cout << summary.FullReport() << std::endl;

  // write results to csv file
  std::ofstream out;
  out.open("/root/slam_ws/src/slam_tutorial/data/optimized_poses.csv");
  for(auto it = ceres_rotations.begin(); it != ceres_rotations.end(); ++it) {
    const auto id = it->first;
    const auto rotation = it->second;
    const auto position = ceres_positions.at(id);

    out << id << ", "
        << rotation[0] << ", " << rotation[1] << ", " 
        << rotation[2] << ", " << rotation[3] << ", "
        << position[0] << ", " << position[2] << ", " << position[3] 
        << "\n";
  }
  out.close();
}

int main(int argc, char** argv) {

  run();
  return 0;
}
