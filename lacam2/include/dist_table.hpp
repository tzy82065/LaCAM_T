/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "orientation.hpp"
#include "utils.hpp"
#include <queue>
#include <tuple> // for std::pair/tuple

// struct DistTable {
//   const uint V_size;  // number of vertices
//   std::vector<std::vector<uint> >
//       table;          // distance table, index: agent-id & vertex-id
//   std::vector<std::queue<Vertex*> > OPEN;  // search queue

//   inline uint get(uint i, uint v_id);      // agent, vertex-id
//   uint get(uint i, Vertex* v);             // agent, vertex

//   DistTable(const Instance& ins);
//   DistTable(const Instance* ins);

//   void setup(const Instance* ins);  // initialization
// };

struct DistTable {
  const uint V_size;  // number of vertices
  // table[agent_id][vertex_id * 4 + orientation]
  const uint width; // 新增：保存地图宽度用于坐标计算
  std::vector<std::vector<uint>> table;
  // OPEN list now stores pairs of (Vertex*, Orientation)
  // 使用 vector<queue> 是为了每个智能体维护一个独立的搜索队列
  std::vector<std::queue<std::pair<Vertex*, Orientation>>> OPEN;

  DistTable(const Instance& ins);
  DistTable(const Instance* ins);
  
  void createDistanceTableWithOrientation(const Instance* ins);

  void setup(const Instance* ins);

  // 获取距离：根据 Agent ID 和 当前状态(位置+方向)
  uint get(uint i, const State& s); 
  // 重载：手动指定位置和方向
  uint get(uint i, Vertex* v, Orientation o);

  // 辅助函数：计算 table 的索引
  inline uint get_index(Vertex* v, Orientation o) const {
    return v->id * 4 + static_cast<uint>(o);
  }
};
