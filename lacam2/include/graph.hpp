/*
 * graph definition
 */
#pragma once
#include "utils.hpp"
#include "orientation.hpp" // 新增

struct Vertex {
  const uint id;     // index for V in Graph
  const uint index;  // index for U, width * y + x, in Graph
  std::vector<Vertex*> neighbor;

  Vertex(uint _id, uint _index);
};
using Vertices = std::vector<Vertex*>;

// 新增：State 结构体，包含位置和方向
struct State {
  Vertex* v;
  Orientation o;

  bool operator==(const State& other) const {
    return v == other.v && o == other.o;
  }
  bool operator!=(const State& other) const {
    return !(*this == other);
  }
};

using Config = std::vector<State>;  // 修改： a set of states for all agents

struct Graph {
  Vertices V;                          // without nullptr
  Vertices U;                          // with nullptr
  uint width;                          // grid width
  uint height;                         // grid height
  Graph();
  Graph(const std::string& filename);  // taking map filename
  ~Graph();

  uint size() const;  // the number of vertices
};

bool is_same_config(
    const Config& C1,
    const Config& C2);  // check equivalence of two configurations

// 新增：仅比较位置（忽略方向）
bool is_same_config_pos(const Config& C1, const Config& C2);

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config& C) const {
    uint hash = C.size();
    for (auto& s : C) {
      auto v_hash = s.v->id;
      auto o_hash = static_cast<uint>(s.o); // 必须包含方向
      uint state_hash = (v_hash << 2) ^ o_hash;
      hash ^= state_hash + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
  }
};

std::ostream& operator<<(std::ostream& os, const Vertex* v);
// 新增：State 的输出重载
std::ostream& operator<<(std::ostream& os, const State& s);
std::ostream& operator<<(std::ostream& os, const Config& config);
