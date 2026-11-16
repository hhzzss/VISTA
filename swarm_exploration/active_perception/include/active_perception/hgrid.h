#ifndef _HGRID_H_
#define _HGRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;


class RayCaster;

namespace fast_planner {

class EDTEnvironment;
class Astar;
class GridInfo;
class UniformGrid;

// struct GridInfo {};

// Hierarchical grid, contains two levels currently
class HGrid {

public:
  HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~HGrid();

  void updateGridData(const int& drone_id, std::vector<int>& grid_ids, bool reallocated,
      const std::vector<int>& last_grid_ids, std::vector<int>& first_ids, std::vector<int>& second_ids);

  bool updateBaseCoor();
  void inputFrontiers(const std::vector<Eigen::Vector3d>& avgs);
  void getCostMatrix(const std::vector<Eigen::Vector3d>& positions,
      const std::vector<Eigen::Vector3d>& velocities, const std::vector<std::vector<int>>& first_ids,
      const std::vector<std::vector<int>>& second_ids, const std::vector<int>& grid_ids, Eigen::MatrixXd& mat);
  void getGridTour(const std::vector<int>& ids, const Eigen::Vector3d& pos,
      std::vector<Eigen::Vector3d>& tour, std::vector<Eigen::Vector3d>& tour2);
  void getFrontiersInGrid(const std::vector<int>& grid_ids, std::vector<int>& ftr_ids);
  bool getNextGrid(const std::vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw);
  void getConsistentGrid(const std::vector<int>& last_ids, const std::vector<int>& cur_ids,
      std::vector<int>& first_ids, std::vector<int>& second_ids);

  void getGridMarker(std::vector<Eigen::Vector3d>& pts1, std::vector<Eigen::Vector3d>& pts2);
  void getGridMarker2(std::vector<Eigen::Vector3d>& pts, std::vector<std::string>& texts);
  void checkFirstGrid(const int& id);
  int getUnknownCellsNum(const int& grid_id);
  Eigen::Vector3d getCenter(const int& grid_id);
  void getActiveGrids(std::vector<int>& grid_ids);
  bool isConsistent(const int& id1, const int& id2);
  double getCostDroneToGrid(
      const Eigen::Vector3d& pos, const int& grid_id, const std::vector<int>& first);
  double getCostGridToGrid(const int& id1, const int& id2, const std::vector<std::vector<int>>& firsts,
      const std::vector<std::vector<int>>& seconds, const int& drone_num);
  unique_ptr<Astar> path_finder_;

private:
  void coarseToFineId(const int& coarse, std::vector<int>& fines);
  void fineToCoarseId(const int& fine, int& coarse);
  GridInfo& getGrid(const int& id);

  bool isClose(const int& id1, const int& id2);
  bool inSameLevel1(const int& id1, const int& id2);

  unique_ptr<UniformGrid> grid1_;  // Coarse level
  unique_ptr<UniformGrid> grid2_;  // Fine level

  shared_ptr<EDTEnvironment> edt_;
  double consistent_cost_;
  double consistent_cost2_;

  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;
  double w_first_;
};

}  // namespace fast_planner
#endif