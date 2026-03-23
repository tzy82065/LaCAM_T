#include "../include/instance.hpp"

Instance::Instance(const std::string& map_filename,
                   const std::vector<uint>& start_indexes,
                   const std::vector<uint>& goal_indexes)
    : G(map_filename),
      starts(Config()),
      goals(Config()),
      N(start_indexes.size())
{
  // for (auto k : start_indexes) starts.push_back(G.U[k]);
  // for (auto k : goal_indexes) goals.push_back(G.U[k]);
  for (auto k : start_indexes) {
      // 修改：推入 State{v, o}
      starts.push_back(State{G.U[k], Orientation::Y_MINUS}); 
  }
  for (auto k : goal_indexes) {
      // 修改：推入 State{v, o}
      // 注意：这里假设目标也是 Y_MINUS。如果目标方向不重要，后续可能需要修改 is_same_config 的逻辑来忽略目标方向。
      // 但现在按要求我们统一设为 Y_MINUS。
      goals.push_back(State{G.U[k], Orientation::Y_MINUS}); 
  }
}

// for load instance
static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

Instance::Instance(const std::string& scen_filename,
                   const std::string& map_filename, const uint _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // load start-goal pairs
  std::ifstream file(scen_filename);
  if (!file) {
    info(0, 0, scen_filename, " is not found");
    return;
  }
  std::string line;
  std::smatch results;

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_instance)) {
      uint x_s = std::stoi(results[1].str());
      uint y_s = std::stoi(results[2].str());
      uint x_g = std::stoi(results[3].str());
      uint y_g = std::stoi(results[4].str());
      if (x_s < 0 || G.width <= x_s || x_g < 0 || G.width <= x_g) break;
      if (y_s < 0 || G.height <= y_s || y_g < 0 || G.height <= y_g) break;
      auto s = G.U[G.width * y_s + x_s];
      auto g = G.U[G.width * y_g + x_g];
      if (s == nullptr || g == nullptr) break;
      // starts.push_back(s);
      // goals.push_back(g);
      starts.push_back(State{s, Orientation::Y_MINUS});
      goals.push_back(State{g, Orientation::Y_MINUS});
    }

    if (starts.size() == N) break;
  }
}

Instance::Instance(const std::string& map_filename, std::mt19937* MT,
                   const uint _N)
    : G(Graph(map_filename)), starts(Config()), goals(Config()), N(_N)
{
  // random assignment
  const auto V_size = G.size();

  // set starts
  auto s_indexes = std::vector<uint>(V_size);
  std::iota(s_indexes.begin(), s_indexes.end(), 0);
  std::shuffle(s_indexes.begin(), s_indexes.end(), *MT);
  size_t i = 0;
  while (true) {
    if (i >= V_size) return;
    //starts.push_back(G.V[s_indexes[i]]);
    starts.push_back(State{G.V[s_indexes[i]], Orientation::Y_MINUS});
    if (starts.size() == N) break;
    ++i;
  }

  // set goals
  auto g_indexes = std::vector<uint>(V_size);
  std::iota(g_indexes.begin(), g_indexes.end(), 0);
  std::shuffle(g_indexes.begin(), g_indexes.end(), *MT);
  size_t j = 0;
  while (true) {
    if (j >= V_size) return;
    //goals.push_back(G.V[g_indexes[j]]);
    goals.push_back(State{G.V[g_indexes[j]], Orientation::Y_MINUS});
    if (goals.size() == N) break;
    ++j;
  }
}

bool Instance::is_valid(const int verbose) const
{
  if (N != starts.size() || N != goals.size()) {
    info(1, verbose, "invalid N, check instance");
    return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& os, const Solution& solution)
{
  auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) {
    os << std::setw(5) << i << ":";
    for (size_t k = 0; k < solution[i].size(); ++k) {
      if (k > 0) os << "->";
      //os << std::setw(5) << solution[i][k]->index;
      os << std::setw(5) << solution[k][i].v->index;
    }
    os << std::endl;
  }
  return os;
}
