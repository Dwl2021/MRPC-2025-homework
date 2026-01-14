#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void Astarpath::begin_grid_map(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GRID_X_SIZE = max_x_id;
  GRID_Y_SIZE = max_y_id;
  GRID_Z_SIZE = max_z_id;
  GLYZ_SIZE = GRID_Y_SIZE * GRID_Z_SIZE;
  GLXYZ_SIZE = GRID_X_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  data_raw = new uint8_t[GLXYZ_SIZE];
  memset(data_raw, 0, GLXYZ_SIZE * sizeof(uint8_t));

  Map_Node = new MappingNodePtr **[GRID_X_SIZE];
  for (int i = 0; i < GRID_X_SIZE; i++) {
    Map_Node[i] = new MappingNodePtr *[GRID_Y_SIZE];
    for (int j = 0; j < GRID_Y_SIZE; j++) {
      Map_Node[i][j] = new MappingNodePtr[GRID_Z_SIZE];
      for (int k = 0; k < GRID_Z_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        Map_Node[i][j][k] = new MappingNode(tmpIdx, pos);
      }
    }
  }
}

void Astarpath::resetGrid(MappingNodePtr ptr) {
  ptr->id = 0;
  ptr->Father = NULL;
  ptr->g_score = inf;
  ptr->f_score = inf;
}

void Astarpath::resetUsedGrids() {
  for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++)
        resetGrid(Map_Node[i][j][k]);
}

void Astarpath::set_barrier(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  data_raw[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;

  if (idx_x == 0 || idx_y == 0 || idx_z == GRID_Z_SIZE || idx_x == GRID_X_SIZE ||
      idx_y == GRID_Y_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GRID_Z_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GRID_Z_SIZE + idx_z] = 1;
  }
}

vector<Vector3d> Astarpath::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GRID_X_SIZE; i++)
    for (int j = 0; j < GRID_Y_SIZE; j++)
      for (int k = 0; k < GRID_Z_SIZE; k++) {
        // if(Map_Node[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (Map_Node[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(Map_Node[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d Astarpath::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i Astarpath::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GRID_X_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GRID_Y_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GRID_Z_SIZE - 1);

  return idx;
}

Vector3i Astarpath::c2i(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GRID_X_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GRID_Y_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GRID_Z_SIZE - 1);

  return idx;
}

Eigen::Vector3d Astarpath::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool Astarpath::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

bool Astarpath::is_occupy(const Eigen::Vector3i &index) {
  return isOccupied(index(0), index(1), index(2));
}

bool Astarpath::is_occupy_raw(const Eigen::Vector3i &index) {
  int idx_x = index(0);
  int idx_y = index(1);
  int idx_z = index(2);
  return (idx_x >= 0 && idx_x < GRID_X_SIZE && idx_y >= 0 && idx_y < GRID_Y_SIZE &&
          idx_z >= 0 && idx_z < GRID_Z_SIZE &&
          (data_raw[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] == 1));
}

inline bool Astarpath::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool Astarpath::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GRID_X_SIZE && idx_y >= 0 && idx_y < GRID_Y_SIZE &&
          idx_z >= 0 && idx_z < GRID_Z_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] == 1));
}

inline bool Astarpath::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GRID_X_SIZE && idx_y >= 0 && idx_y < GRID_Y_SIZE &&
          idx_z >= 0 && idx_z < GRID_Z_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GRID_Z_SIZE + idx_z] < 1));
}

inline void Astarpath::AstarGetSucc(MappingNodePtr currentPtr,
                                          vector<MappingNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i Idx_neighbor;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        Idx_neighbor(0) = (currentPtr->index)(0) + dx;
        Idx_neighbor(1) = (currentPtr->index)(1) + dy;
        Idx_neighbor(2) = (currentPtr->index)(2) + dz;

        if (Idx_neighbor(0) < 0 || Idx_neighbor(0) >= GRID_X_SIZE ||
            Idx_neighbor(1) < 0 || Idx_neighbor(1) >= GRID_Y_SIZE ||
            Idx_neighbor(2) < 0 || Idx_neighbor(2) >= GRID_Z_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            Map_Node[Idx_neighbor(0)][Idx_neighbor(1)][Idx_neighbor(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

  double Astarpath::getHeu(MappingNodePtr node1, MappingNodePtr node2) {
    double dx = std::abs(node1->coord(0) - node2->coord(0));
    double dy = std::abs(node1->coord(1) - node2->coord(1));
    double dz = std::abs(node1->coord(2) - node2->coord(2));

    // --- 优化 A: 26邻域对角线距离
    double min_xyz = std::min({dx, dy, dz});
    double max_xyz = std::max({dx, dy, dz});
    double mid_xyz = dx + dy + dz - min_xyz - max_xyz;

    // 26邻域下，最简化的启发式是直接取最大轴向距离（Chebyshev 距离的变形）
    // 或者使用精确的对角线系数：1, 1.414(sqrt2), 1.732(sqrt3)
    double heu = (1.732 - 1.414) * min_xyz + (1.414 - 1.0) * mid_xyz + 1.0 * max_xyz;

    // --- 优化 B: 动态 Tie-breaker (解决转折多、搜索慢的问题) ---
    // 计算当前点到“起点-终点”直线的叉积偏置
    // 假设 start_coord 是搜索起始点的坐标，我们可以通过传入或存储它来增加指向性
    // 如果不方便获取起点，可以调大你现有的 tie_breaker 系数
    double p_tie_breaker = 1.0 + 0.005; // 稍微从 0.0001 调大到 0.005
    
    return p_tie_breaker * heu;
}

bool Astarpath::AstarSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // start_point 和 end_point 索引
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  //start_point 和 end_point 的位置
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // 初始化 struct MappingNode 的指针，分别代表 start node 和 goal node
  // 
  MappingNodePtr startPtr = new MappingNode(start_idx, start_pt);
  MappingNodePtr endPtr = new MappingNode(end_idx, end_pt);

  // Openset 是通过 STL 库中的 multimap 实现的open_list
  Openset.clear();
  // currentPtr 表示 open_list 中 f（n） 最低的节点
  MappingNodePtr currentPtr = NULL;
  MappingNodePtr neighborPtr = NULL;

  // 将 Start 节点放在 Open Set 中
  startPtr->g_score = 0;
  /**
   *
   * STEP 1.1:  完成 Astarpath::getHeu
   *
   * **/



  startPtr->f_score = getHeu(startPtr, endPtr);

  

  startPtr->id = 1;
  startPtr->coord = start_pt;
  startPtr -> Father = NULL;
  Openset.insert(make_pair(startPtr->f_score, startPtr));


  double tentative_g_score;
  vector<MappingNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.2:  完成循环
   *
   * **/

  while (!Openset.empty()) {
    currentPtr = Openset.begin()->second;
    Openset.erase(Openset.begin());
    currentPtr->id = -1; 

    if(currentPtr->index == end_idx) { 
        terminatePtr = currentPtr;
        return true;
    }

    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

    for(unsigned int i=0; i < neighborPtrSets.size(); i++) {
        neighborPtr = neighborPtrSets[i];
        if(neighborPtr->id == -1 || isOccupied(neighborPtr->index)) continue;

        // 1. 转向惩罚
        double turn_penalty = 0.0;
        if (currentPtr->Father != NULL) {
            Eigen::Vector3i dir_last = currentPtr->index - currentPtr->Father->index;
            Eigen::Vector3i dir_now = neighborPtr->index - currentPtr->index;
            if (dir_last != dir_now) {
                turn_penalty = 4.0; 
            }
        }

        double safety_margin_cost = 0.0;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (isOccupied(neighborPtr->index + Eigen::Vector3i(dx, dy, 0))) {
                    safety_margin_cost = 2.0;
                    break;
                }
            }
        }

        double tentative_g_score = currentPtr->g_score + edgeCostSets[i] + turn_penalty + safety_margin_cost;

        if(neighborPtr->id <= 0 || tentative_g_score < neighborPtr->g_score) {
            // 如果已经在 Openset 中，更新它
            if (neighborPtr->id == 1) {
                auto range = Openset.equal_range(neighborPtr->f_score);
                for (auto it = range.first; it != range.second; ++it) {
                    if (it->second == neighborPtr) {
                        Openset.erase(it);
                        break;
                    }
                }
            }

            neighborPtr->g_score = tentative_g_score;
            neighborPtr->Father = currentPtr;
            // 3. Tie-breaker：增加目标指向性
            neighborPtr->f_score = tentative_g_score + (1.0 + 0.001) * getHeu(neighborPtr, endPtr);
            neighborPtr->id = 1;
            Openset.insert(std::make_pair(neighborPtr->f_score, neighborPtr));
        }
    }
}

  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  return false;
}


vector<Vector3d> Astarpath::getPath() {
  vector<Vector3d> path;
  vector<MappingNodePtr> front_path;
do
{
terminatePtr->coord=gridIndex2coord(terminatePtr->index);
front_path.push_back(terminatePtr);
terminatePtr=terminatePtr->Father;
}while(terminatePtr->Father!=NULL);
  /**
   *
   * STEP 1.3:  追溯找到的路径
   *
   * **/

  // ???
  front_path.push_back(terminatePtr);
  for (int i = front_path.size() - 1; i >= 0; i--) {
    path.push_back(front_path[i]->coord);
  }

  return path;
}


std::vector<Vector3d> Astarpath::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {

  //init
  double dmax=0,d;
  int index=0;
  int end = path.size();
  //1.计算距离首尾连成直线最大的点，并将点集从此处分开
  for(int i=1;i<end-1;i++)
  {
    d=perpendicularDistance(path[i],path[0],path[end-1]);
    if(d>dmax)
    {
      index=i;
      dmax=d;
    }
  }
  vector<Vector3d> subPath1;
  int j = 0;
  while(j<index+1){
    subPath1.push_back(path[j]);
    j++;
  }
  vector<Vector3d> subPath2;
   while(j<int(path.size())){
    subPath2.push_back(path[j]);
    j++;
  }
  //2.拆分点集
  vector<Vector3d> recPath1;
  vector<Vector3d> recPath2;
  vector<Vector3d> resultPath;
  if(dmax>path_resolution)
  {
    recPath1=pathSimplify(subPath1,path_resolution);
    recPath2=pathSimplify(subPath2,path_resolution);
   for(int i=0;i<int(recPath1.size());i++){
    resultPath.push_back(recPath1[i]);
  }
     for(int i=0;i<int(recPath2.size());i++){
    resultPath.push_back(recPath2[i]);
  }
  }else{
    if(path.size()>1){
      resultPath.push_back(path[0]);
      resultPath.push_back(path[end-1]);
    }else{
      resultPath.push_back(path[0]);
    }
    
  }

  return resultPath;
}

double Astarpath::perpendicularDistance(const Eigen::Vector3d point_insert,const Eigen:: Vector3d point_st,const Eigen::Vector3d point_end)
{
  Vector3d line1=point_end-point_st;
  Vector3d line2=point_insert-point_st;
  return double(line2.cross(line1).norm()/line1.norm());
}

Vector3d Astarpath::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}


int Astarpath::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe

  double delta_t=resolution/1.0;//conservative advance step size;
  double t = delta_t;
  Vector3d advancePos;
  for(int i=0;i<polyCoeff.rows();i++)
  {
    while(t<time(i)){
     advancePos=getPosPoly(polyCoeff,i,t) ;
     if(isOccupied(coord2gridIndex(advancePos))){
       unsafe_segment=i;
       break;
     }   
     t+=delta_t;
    }
    if(unsafe_segment!=-1){

      break;
    }else{
      t=delta_t;
    }
  }
  return unsafe_segment;
}

void Astarpath::resetOccupy(){
  for (int i = 0; i < GRID_X_SIZE; i++)
for (int j = 0; j < GRID_Y_SIZE; j++)
  for (int k = 0; k < GRID_Z_SIZE; k++) {
    data[i * GLYZ_SIZE + j * GRID_Z_SIZE + k] = 0;
    data_raw[i * GLYZ_SIZE + j * GRID_Z_SIZE + k] = 0;
  }
}
