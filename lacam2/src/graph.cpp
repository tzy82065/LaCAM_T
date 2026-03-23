#include "../include/graph.hpp"

Vertex::Vertex(uint _id, uint _index)
    : id(_id), index(_index), neighbor(Vertices())
{
}

Graph::Graph() : V(Vertices()), width(0), height(0) {}
Graph::~Graph()
{
  for (auto& v : V)
    if (v != nullptr) delete v;
  V.clear();
}

// to load graph
static const std::regex r_height = std::regex(R"(height\s(\d+))");
static const std::regex r_width = std::regex(R"(width\s(\d+))");
static const std::regex r_map = std::regex(R"(map)");

Graph::Graph(const std::string& filename) : V(Vertices()), width(0), height(0)
{
  std::ifstream file(filename);
  if (!file) {
    std::cout << "file " << filename << " is not found." << std::endl;
    return;
  }
  std::string line;
  std::smatch results;

  // read fundamental graph parameters
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_height)) {
      height = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_width)) {
      width = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_map)) break;
  }

  U = Vertices(width * height, nullptr);

  // create vertices
  uint y = 0;
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();
    for (uint x = 0; x < width; ++x) {
      char s = line[x];
      if (s == 'T' or s == '@') continue;  // object
      auto index = width * y + x;
      auto v = new Vertex(V.size(), index);
      V.push_back(v);
      U[index] = v;
    }
    ++y;
  }
  file.close();

  // create edges
  for (uint y = 0; y < height; ++y) {
    for (uint x = 0; x < width; ++x) {
      auto v = U[width * y + x];
      if (v == nullptr) continue;
      // left
      if (x > 0) {
        auto u = U[width * y + (x - 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // right
      if (x < width - 1) {
        auto u = U[width * y + (x + 1)];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // up
      if (y < height - 1) {
        auto u = U[width * (y + 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
      // down
      if (y > 0) {
        auto u = U[width * (y - 1) + x];
        if (u != nullptr) v->neighbor.push_back(u);
      }
    }
  }
}

uint Graph::size() const { return V.size(); }

// bool is_same_config(const Config& C1, const Config& C2)
// {
//   const auto N = C1.size();
//   for (size_t i = 0; i < N; ++i) {
//     if (C1[i]->id != C2[i]->id) return false;
//   }
//   return true;
// }

// is_same_config 实现其实可以复用 vector 的 == 操作符，但为了兼容旧代码显式保留
bool is_same_config(const Config& C1, const Config& C2)
{
  const auto N = C1.size();
  for (size_t i = 0; i < N; ++i) {
    if (C1[i] != C2[i]) return false; // 使用 State 的 != 重载
  }
  return true;
}

// 新增：仅比较位置
bool is_same_config_pos(const Config& C1, const Config& C2)
{
  const auto N = C1.size();
  for (size_t i = 0; i < N; ++i) {
    // 关键修改：只比较 Vertex 指针（或 index）
    if (C1[i].v != C2[i].v) return false;
  }
  return true;
}

// uint ConfigHasher::operator()(const Config& C) const
// {
//   uint hash = C.size();
//   for (auto& v : C) hash ^= v->id + 0x9e3779b9 + (hash << 6) + (hash >> 2);
//   return hash;
// }

// [修正] ConfigHasher 必须同时哈希位置和方向
// uint ConfigHasher::operator()(const Config& C) const
// {
//   uint hash = C.size();
//   for (auto& s : C) {
//     // 混合位置 ID
//     auto v_hash = s.v->id;
//     // 混合方向 (假设 Orientation 转为 int 是 0-3)
//     auto o_hash = static_cast<uint>(s.o);
    
//     // 组合哈希 (使用位移避免冲突)
//     uint state_hash = (v_hash << 2) ^ o_hash; 
    
//     hash ^= state_hash + 0x9e3779b9 + (hash << 6) + (hash >> 2);
//   }
//   return hash;
// }

std::ostream& operator<<(std::ostream& os, const Vertex* v)
{
  os << v->index;
  return os;
}

// 新增
std::ostream& operator<<(std::ostream& os, const State& s)
{
  os << "(" << s.v->index << "," << orientationToString(s.o) << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Config& config)
{
  os << "<";
  const auto N = config.size();
  for (size_t i = 0; i < N; ++i) {
    if (i > 0) os << ",";
    os << std::setw(5) << config[i];
  }
  os << ">";
  return os;
}
