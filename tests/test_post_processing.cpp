#include <lacam2.hpp>
#include "../include/orientation.hpp" // [新增] 引入方向定义
#include "gtest/gtest.h"

TEST(PostProcesing, validate)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto start_indexes = std::vector<uint>({0, 8});
  const auto goal_indexes = std::vector<uint>({9, 1});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(3);
  // [修正] 使用 State{v, o} 初始化 Config
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[8], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[1], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[9], Orientation::Y_MINUS}, State{ins.G.U[1], Orientation::Y_MINUS}});
  ASSERT_TRUE(is_feasible_solution(ins, sol));

  // invalid start
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[4], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[1], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[9], Orientation::Y_MINUS}, State{ins.G.U[1], Orientation::Y_MINUS}});
  ASSERT_FALSE(is_feasible_solution(ins, sol));

  // invalid goal
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[8], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[1], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  ASSERT_FALSE(is_feasible_solution(ins, sol));

  // invalid transition
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[8], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[4], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[9], Orientation::Y_MINUS}, State{ins.G.U[1], Orientation::Y_MINUS}});
  ASSERT_FALSE(is_feasible_solution(ins, sol));

  // swap conflict
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[8], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[8], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[9], Orientation::Y_MINUS}, State{ins.G.U[1], Orientation::Y_MINUS}});
  ASSERT_FALSE(is_feasible_solution(ins, sol));

  // vertex conflict
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[8], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[0], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[8], Orientation::Y_MINUS}, State{ins.G.U[1], Orientation::Y_MINUS}});
  sol.push_back(Config({State{ins.G.U[9], Orientation::Y_MINUS}, State{ins.G.U[1], Orientation::Y_MINUS}}));
  ASSERT_FALSE(is_feasible_solution(ins, sol));
}

TEST(PostProcessing, metrics)
{
  const auto map_filename = "./assets/empty-8-8.map";
  const auto start_indexes = std::vector<uint>({0, 5, 10});
  const auto goal_indexes = std::vector<uint>({2, 4, 11});
  const auto ins = Instance(map_filename, start_indexes, goal_indexes);

  // correct solution
  auto sol = Solution(3);
  // [修正] 使用 State{v, o} 初始化
  sol[0] = Config({State{ins.G.U[0], Orientation::Y_MINUS}, State{ins.G.U[5], Orientation::Y_MINUS}, State{ins.G.U[10], Orientation::Y_MINUS}});
  sol[1] = Config({State{ins.G.U[1], Orientation::Y_MINUS}, State{ins.G.U[4], Orientation::Y_MINUS}, State{ins.G.U[11], Orientation::Y_MINUS}});
  sol[2] = Config({State{ins.G.U[2], Orientation::Y_MINUS}, State{ins.G.U[4], Orientation::Y_MINUS}, State{ins.G.U[11], Orientation::Y_MINUS}});

  ASSERT_EQ(get_makespan(sol), 2);
  ASSERT_EQ(get_path_cost(sol, 0), 2);
  ASSERT_EQ(get_path_cost(sol, 1), 1);
  ASSERT_EQ(get_path_cost(sol, 2), 1);
  ASSERT_EQ(get_sum_of_costs(sol), 4);
  ASSERT_EQ(get_sum_of_loss(sol), 4);
}