/*
 * src/dist_table.cpp
 */
#include "../include/dist_table.hpp"

DistTable::DistTable(const Instance& ins)
    : V_size(ins.G.V.size()),
      width(ins.G.width),
      table(ins.N, std::vector<uint>(V_size * 4, V_size * 4))
{
  //setup(&ins);
  createDistanceTableWithOrientation(&ins);
}

DistTable::DistTable(const Instance* ins)
    : V_size(ins->G.V.size()),
      width(ins->G.width),
      table(ins->N, std::vector<uint>(V_size * 4, V_size * 4))
{
  //setup(ins);
  createDistanceTableWithOrientation(ins);
}

void DistTable::createDistanceTableWithOrientation(const Instance* ins)
{
  // 1. 初始化表格为最大值 (V_size * 4 是因为每个节点有4个方向)
  for (auto& row : table) {
    std::fill(row.begin(), row.end(), V_size * 4);
  }

  // 2. 遍历每个智能体进行反向 BFS
  for (size_t i = 0; i < ins->N; ++i) {
    // 使用本地队列进行完全搜索 (Eager BFS)
    std::queue<std::pair<Vertex*, Orientation>> Q;
    
    // 获取目标位置
    auto goal_v = ins->goals[i].v; // 目标位置
    
    // 初始化目标状态：目标位置的【所有方向】距离都设为 0
    // 意味着只要到达目标位置，无论朝向如何都视为完成任务
    for (int d = 0; d < 4; ++d) {
      Orientation dir = static_cast<Orientation>(d);
      uint idx = get_index(goal_v, dir);
      table[i][idx] = 0;
      Q.push({goal_v, dir});
    }

    // 开始 BFS
    while (!Q.empty()) {
      auto [u, dir_u] = Q.front();
      Q.pop();

      uint idx_u = get_index(u, dir_u);
      uint d_curr = table[i][idx_u];

      // --- 动作 1: 逆向旋转 (Reverse Rotation) ---
      // 如果我们在 (u, dir_prev) 可以旋转到达 (u, dir_u)，那么 distance(u, dir_prev) = d_curr + 1
      // 可能的前驱方向是 dir_u 的左右 90 度
      std::vector<Orientation> prev_dirs;
      if (dir_u == Orientation::X_PLUS || dir_u == Orientation::X_MINUS) {
          prev_dirs = {Orientation::Y_PLUS, Orientation::Y_MINUS};
      } else {
          prev_dirs = {Orientation::X_PLUS, Orientation::X_MINUS};
      }

      for (auto dir_prev : prev_dirs) {
          uint idx_prev = get_index(u, dir_prev);
          // 如果发现了更短的路径（或者之前未访问过）
          if (table[i][idx_prev] > d_curr + 1) {
              table[i][idx_prev] = d_curr + 1;
              Q.push({u, dir_prev});
          }
      }

      // --- 动作 2: 逆向移动 (Reverse Move) ---
      // 如果我们从 (v, dir_u) 向前移动一步能到达 (u, dir_u)
      // 那么 distance(v, dir_u) = d_curr + 1
      // 这要求：从 v 移动到 u 的方向必须等于 dir_u
      
      for (auto v : u->neighbor) {
          // 计算 v -> u 的方向 (注意：是 v 指向 u)
          int diff = (int)u->index - (int)v->index;
          Orientation move_dir;
          bool valid_dir = false;

          // 假设 width 是地图宽度
          if (diff == 1) { move_dir = Orientation::X_PLUS; valid_dir = true; }        // v(x) -> u(x+1) : Right
          else if (diff == -1) { move_dir = Orientation::X_MINUS; valid_dir = true; } // v(x) -> u(x-1) : Left
          else if (diff == (int)width) { move_dir = Orientation::Y_PLUS; valid_dir = true; }   // v(y) -> u(y+1) : Down
          else if (diff == -(int)width) { move_dir = Orientation::Y_MINUS; valid_dir = true; } // v(y) -> u(y-1) : Up
          
          if (!valid_dir) continue;

          // 只有当 v -> u 的方向与当前朝向 dir_u 一致时，v 才是合法的上一步
          if (move_dir == dir_u) {
              uint idx_prev = get_index(v, dir_u);
              if (table[i][idx_prev] > d_curr + 1) {
                  table[i][idx_prev] = d_curr + 1;
                  Q.push({v, dir_u});
              }
          }
      }
    }
  }
}

void DistTable::setup(const Instance* ins)
{
  OPEN.resize(ins->N);
  for (size_t i = 0; i < ins->N; ++i) {
    std::queue<std::pair<Vertex*, Orientation>> empty;
    std::swap(OPEN[i], empty);

    auto goal_state = ins->goals[i];
    
    // 初始化：将终点的 Y_MINUS 方向设为 0
    // [修改] 既然我们强制所有状态都是 Y_MINUS，这里只关注 Y_MINUS 即可
    Orientation o = Orientation::Y_MINUS;
    uint idx = get_index(goal_state.v, o);
      
    table[i][idx] = 0;
    OPEN[i].push({goal_state.v, o});
  }
}

uint DistTable::get(uint i, const State& s) {
    return get(i, s.v, s.o);
}

uint DistTable::get(uint i, Vertex* v_query, Orientation o_query)
{
  // [修正] 使用传入的 o_query，而不是强制 Y_MINUS
  uint idx = get_index(v_query, o_query);
  
  // 直接查表返回
  // 因为 createDistanceTableWithOrientation 是 Eager BFS，所有可达状态此时都已计算完毕
  return table[i][idx];
}