#ifndef GASA_H_
#define GASA_H_
#include <iostream>
#include <map>
#include <vector>
// 目标函数
int objective_function(std::map<std::string, int> paramMap);

// 模拟退火算法
std::map<std::string, int> simulated_annealing(double initial_temperature,
                                               double final_temperature,
                                               double cooling_rate);

extern std::string WHICHMAP;

// 问题参数
const int NUM_PARAMETERS = 10;  // 参数个数
const int PARAMETER_MIN = 0;    // 参数最小值(不用)
const int PARAMETER_MAX = 100;  // 参数最大值(不用)

const int EXTEND =
    20;  // 在退火算法得到的参数左右两边扩展的长度，形成参数范围（要考虑是否超过了原本的范围，如果超过了，就以原本的额范围作为遗传算法的参数范围）

// 遗传算法参数
const int POPULATION_SIZE = 60;  // 种群大小
const int DISCARD_NUM = 10;      // 每一次迭代种群丢弃的个体数量
const int MAX_GENERATIONS = 30;  // 最大迭代代数
const double MUTATION_RATE = 0.01;  // 变异率

// 个体定义
struct Individual {
  std::vector<int> genes;      // 基因序列
  int fitness;                 // 适应度值
  std::vector<int> min_value;  // 记录个体范围的下限
  std::vector<int> max_value;  // 记录个体范围的上限

  Individual() {
    genes.resize(NUM_PARAMETERS);
    min_value.resize(NUM_PARAMETERS);
    max_value.resize(NUM_PARAMETERS);
    fitness = 0;
  }
};

// 生成随机整数
int getRandomInt(int min, int max);

// 初始化种群
void initializePopulation(std::vector<Individual> &population,
                          std::map<std::string, int> &param);

// 计算个体的适应度值
int calculateFitness(std::vector<Individual> &population, int index);

// 评估种群中的个体
void evaluatePopulation(std::vector<Individual> &population);

// 淘汰个体
void discard(std::vector<Individual> &population);

// 选择操作（轮盘赌选择）
Individual selectParent(std::vector<Individual> &population);

// 交叉操作
Individual crossover(const Individual &parent1, const Individual &parent2);

// 变异操作
void mutate(Individual &individual);
// 遗传算法主循环
void geneticAlgorithm(std::vector<Individual> &population);

int gaRun(std::map<std::string, int> param);
#endif