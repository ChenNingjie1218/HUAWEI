#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

#include "gasa.h"

extern std::string WHICHMAP;

// 生成随机整数
int getRandomInt(int min, int max) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dis(min, max);
  return dis(gen);
}

// 初始化种群
void initializePopulation(std::vector<Individual> &population,
                          std::map<std::string, int> &param) {
  std::map<std::string, int> limitMap[NUM_PARAMETERS];
  // std::cout << "最优解ppp" << std::endl;
  int tolerant_time = param["TOLERANT_TIME"];  // 范围[0,400]
  if (tolerant_time - EXTEND > 0) {
    limitMap[0].insert(std::make_pair("min", tolerant_time - EXTEND));
  } else {
    limitMap[0].insert(std::make_pair("min", 0));
  }
  if (tolerant_time + EXTEND < 400) {
    limitMap[0].insert(std::make_pair("max", tolerant_time + EXTEND));
  } else {
    limitMap[0].insert(std::make_pair("max", 400));
  }

  int tolerant_leave_time = param["TOLERANT_LEAVE_TIME"];  // 范围 [0, 200]
  if (tolerant_leave_time - EXTEND > 0) {
    limitMap[1].insert(std::make_pair("min", tolerant_leave_time - EXTEND));
  } else {
    limitMap[1].insert(std::make_pair("min", 0));
  }
  if (tolerant_leave_time + EXTEND < 200) {
    limitMap[1].insert(std::make_pair("max", tolerant_leave_time + EXTEND));
  } else {
    limitMap[1].insert(std::make_pair("max", 200));
  }

  int goods_value_value = param["GOODS_VALUE_VALVE"];  // 范围[0, 200]
  if (goods_value_value - EXTEND > 0) {
    limitMap[2].insert(std::make_pair("min", goods_value_value - EXTEND));
  } else {
    limitMap[2].insert(std::make_pair("min", 0));
  }
  if (goods_value_value + EXTEND < 200) {
    limitMap[2].insert(std::make_pair("max", goods_value_value + EXTEND));
  } else {
    limitMap[2].insert(std::make_pair("max", 200));
  }

  int goods_filter_value_num =
      param["GOODS_FILTER_VALVE_NUM"];  // 范围[0, 2000]
  if (goods_filter_value_num - EXTEND > 0) {
    limitMap[3].insert(std::make_pair("min", goods_filter_value_num - EXTEND));
  } else {
    limitMap[3].insert(std::make_pair("min", 0));
  }
  if (goods_filter_value_num + EXTEND < 2000) {
    limitMap[3].insert(std::make_pair("max", goods_filter_value_num + EXTEND));
  } else {
    limitMap[3].insert(std::make_pair("max", 2000));
  }

  int valueable_goods_value = param["VALUEABLE_GOODS_VALVE"];  // 范围 [0, 200]
  if (valueable_goods_value - EXTEND > 0) {
    limitMap[4].insert(std::make_pair("min", valueable_goods_value - EXTEND));
  } else {
    limitMap[4].insert(std::make_pair("min", 0));
  }
  if (valueable_goods_value + EXTEND < 200) {
    limitMap[4].insert(std::make_pair("max", valueable_goods_value + EXTEND));
  } else {
    limitMap[4].insert(std::make_pair("max", 200));
  }

  int final_tolerant_time = param["FINAL_TOLERANT_TIME"];  // 范围[0, 5000]
  if (final_tolerant_time - EXTEND > 0) {
    limitMap[5].insert(std::make_pair("min", final_tolerant_time - EXTEND));
  } else {
    limitMap[5].insert(std::make_pair("min", 0));
  }
  if (final_tolerant_time + EXTEND < 5000) {
    limitMap[5].insert(std::make_pair("max", final_tolerant_time + EXTEND));
  } else {
    limitMap[5].insert(std::make_pair("max", 5000));
  }

  int busy_value = param["BUSY_VALVE"];  // 范围[0, 10]
  if (busy_value - EXTEND > 0) {
    limitMap[6].insert(std::make_pair("min", busy_value - EXTEND));
  } else {
    limitMap[6].insert(std::make_pair("min", 0));
  }
  if (busy_value + EXTEND < 10) {
    limitMap[6].insert(std::make_pair("max", busy_value + EXTEND));
  } else {
    limitMap[6].insert(std::make_pair("max", 10));
  }

  int boat_capacity_reduce = param["BOAT_CAPACITY_REDUCE"];  // 范围[0, 30]
  if (boat_capacity_reduce - EXTEND > 0) {
    limitMap[7].insert(std::make_pair("min", boat_capacity_reduce - EXTEND));
  } else {
    limitMap[7].insert(std::make_pair("min", 0));
  }
  if (boat_capacity_reduce + EXTEND < 30) {
    limitMap[7].insert(std::make_pair("max", boat_capacity_reduce + EXTEND));
  } else {
    limitMap[7].insert(std::make_pair("max", 30));
  }

  int max_robot_num_1 = param["MAX_ROBOT_NUM_1"];  // 范围[1, 25]
  if (max_robot_num_1 - EXTEND > 1) {
    limitMap[8].insert(std::make_pair("min", max_robot_num_1 - EXTEND));
  } else {
    limitMap[8].insert(std::make_pair("min", 1));
  }
  if (max_robot_num_1 + EXTEND < 25) {
    limitMap[8].insert(std::make_pair("max", max_robot_num_1 + EXTEND));
  } else {
    limitMap[8].insert(std::make_pair("max", 25));
  }

  int max_robot_num_2 = param["MAX_ROBOT_NUM_2"];  // 范围[1, 25]
  if (max_robot_num_2 - EXTEND > 1) {
    limitMap[9].insert(std::make_pair("min", max_robot_num_2 - EXTEND));
  } else {
    limitMap[9].insert(std::make_pair("min", 1));
  }
  if (max_robot_num_2 + EXTEND < 25) {
    limitMap[9].insert(std::make_pair("max", max_robot_num_2 + EXTEND));
  } else {
    limitMap[9].insert(std::make_pair("max", 25));
  }

  int max_boat_num = param["MAX_BOAT_NUM"];  // 范围[1, 3]
  if (max_boat_num - EXTEND > 1) {
    limitMap[10].insert(std::make_pair("min", max_boat_num - EXTEND));
  } else {
    limitMap[10].insert(std::make_pair("min", 1));
  }
  if (max_boat_num + EXTEND < 3) {
    limitMap[10].insert(std::make_pair("max", max_boat_num + EXTEND));
  } else {
    limitMap[10].insert(std::make_pair("max", 3));
  }

  int avr_money_differential = param["AVR_MONEY_DIFFERENTIAL"];  // 范围[0, 100]
  if (avr_money_differential - EXTEND > 0) {
    limitMap[11].insert(std::make_pair("min", avr_money_differential - EXTEND));
  } else {
    limitMap[11].insert(std::make_pair("min", 0));
  }
  if (avr_money_differential + EXTEND < 100) {
    limitMap[11].insert(
        std::make_pair("max", avr_money_differential + EXTEND));
  } else {
    limitMap[11].insert(std::make_pair("max", 100));
  }

  int find_neighbor_max_robot = param["FIND_NEIGHBOR_MAX_ROBOT"];  // 范围[0, 10]
  if (find_neighbor_max_robot - EXTEND > 0) {
    limitMap[12].insert(std::make_pair("min", find_neighbor_max_robot - EXTEND));
  } else {
    limitMap[12].insert(std::make_pair("min", 0));
  }
  if (find_neighbor_max_robot + EXTEND < 10) {
    limitMap[12].insert(
        std::make_pair("max", find_neighbor_max_robot + EXTEND));
  } else {
    limitMap[12].insert(std::make_pair("max", 10));
  }
  // std::cout << "yyyyyyyy" << std::endl;
  for (int i = 0; i < POPULATION_SIZE - 1; ++i) {
    Individual individual;
    for (int j = 0; j < NUM_PARAMETERS; ++j) {
      // std::cout << "min = " << limitMap[j]["min"] << " max = " <<
      // limitMap[j]["max"]; std::cout << "yyyyyyyy = " << j << " " <<
      // limitMap[j]["min"] << " " << limitMap[j]["max"] << " " <<
      // individual.genes.size() << " " << individual.min_value.size() << " " <<
      // individual.max_value.size() << std::endl;
      individual.genes[j] =
          getRandomInt(limitMap[j]["min"], limitMap[j]["max"]);
      // std::cout << "测试" << std::endl;
      individual.min_value[j] = limitMap[j]["min"];
      individual.max_value[j] = limitMap[j]["max"];
    }
    population.push_back(individual);
  }
  // std::cout << "最优解" << std::endl;
  // 把退火算法最优解放进去
  Individual individual;
  individual.genes[0] = tolerant_time;
  individual.min_value[0] = limitMap[0]["min"];
  individual.max_value[0] = limitMap[0]["max"];

  individual.genes[1] = tolerant_leave_time;
  individual.min_value[1] = limitMap[1]["min"];
  individual.max_value[1] = limitMap[1]["max"];

  individual.genes[2] = goods_value_value;
  individual.min_value[2] = limitMap[2]["min"];
  individual.max_value[2] = limitMap[2]["max"];

  individual.genes[3] = goods_filter_value_num;
  individual.min_value[3] = limitMap[3]["min"];
  individual.max_value[3] = limitMap[3]["max"];

  individual.genes[4] = valueable_goods_value;
  individual.min_value[4] = limitMap[4]["min"];
  individual.max_value[4] = limitMap[4]["max"];

  individual.genes[5] = final_tolerant_time;
  individual.min_value[5] = limitMap[5]["min"];
  individual.max_value[5] = limitMap[5]["max"];

  individual.genes[6] = busy_value;
  individual.min_value[6] = limitMap[6]["min"];
  individual.max_value[6] = limitMap[6]["max"];

  individual.genes[7] = boat_capacity_reduce;
  individual.min_value[7] = limitMap[7]["min"];
  individual.max_value[7] = limitMap[7]["max"];

  individual.genes[8] = max_robot_num_1;
  individual.min_value[8] = limitMap[8]["min"];
  individual.max_value[8] = limitMap[8]["max"];
  
  individual.genes[9] = max_robot_num_2;
  individual.min_value[9] = limitMap[9]["min"];
  individual.max_value[9] = limitMap[9]["max"];

  individual.genes[10] = max_boat_num;
  individual.min_value[10] = limitMap[10]["min"];
  individual.max_value[10] = limitMap[10]["max"];

  individual.genes[11] = avr_money_differential;
  individual.min_value[11] = limitMap[11]["min"];
  individual.max_value[11] = limitMap[11]["max"];

  individual.genes[12] = find_neighbor_max_robot;
  individual.min_value[12] = limitMap[12]["min"];
  individual.max_value[12] = limitMap[12]["max"];

  population.push_back(individual);
  // for (auto it = individual.genes.begin(); it != individual.genes.end();
  // ++it){
  //   std::cout << *it << " ";
  // }
  // std::cout << std::endl;
}

// 计算个体的适应度值(就是目标函数)
int calculateFitness(std::vector<Individual> &population, int index) {
  // int fitness = 0;
  // for (int i = 0; i < NUM_PARAMETERS; ++i) {
  //   fitness += individual.genes[i];
  // }
  // 计算个体的适应度
  // 写入数据到文件
  auto individual = population[index];
  // std::cout << "测试" << std::endl;
  std::string s = "./gasa/data/data" + std::to_string(index) + ".txt";
  std::ofstream outputFile(
      s.c_str(), std::ios::out | std::ios::trunc);  // 打开文件用于写入
  if (outputFile.is_open()) {
    outputFile << individual.genes[0] << std::endl;
    outputFile << individual.genes[1] << std::endl;
    outputFile << individual.genes[2] << std::endl;
    outputFile << individual.genes[3] << std::endl;
    outputFile << individual.genes[4] << std::endl;
    outputFile << individual.genes[5] << std::endl;
    outputFile << individual.genes[6] << std::endl;
    outputFile << individual.genes[7] << std::endl;
    outputFile << individual.genes[8] << std::endl;
    outputFile << individual.genes[9] << std::endl;
    outputFile << individual.genes[10] << std::endl;
    outputFile << individual.genes[11] << std::endl;
    outputFile << individual.genes[12] << std::endl;
    outputFile << "1" << std::endl;
    outputFile.close();  // 关闭文件
    // std::cout << "写入data成功" << std::endl;
  } else {
    std::cout << "无法打开文件进行写入操作。" << std::endl;
  }
  std::string command =
      "./SemiFinalJudge "
      "'./build/gasa/src/mainparam " +
      std::to_string(index) + "' -m ./doc/" + WHICHMAP + " -r 1.rep -l NONE";
  std::system(command.c_str());
  // 从文件中读取数据
  int money1;
  std::string str = "./gasa/money/money" + std::to_string(index) + ".txt";
  std::fstream inputFile1(str.c_str(),
                          std::ios::in | std::ios::out);  // 打开文件用于读取
  if (inputFile1.is_open()) {
    std::string line;
    while (std::getline(inputFile1, line)) {
      std::cout << line << std::endl;
      money1 = std::stod(line);
    }
    inputFile1.seekp(0, std::ios::beg);  // 定位到文件开头
    inputFile1 << 0;
    inputFile1.close();  // 关闭文件
    std::cout << "读取money成功" << std::endl;
  } else {
    std::cout << "无法打开文件进行读取操作。" << std::endl;
    exit(-1);
  }
  population[index].fitness = money1;
  return 0;
}

// 评估种群中的个体
void evaluatePopulation(std::vector<Individual> &population) {
  // 使用线程计算
  std::vector<std::thread> threads;
  for (int i = 0; i < POPULATION_SIZE; ++i) {
    threads.push_back(std::thread(
        [&population, index = i] { calculateFitness(population, index); }));
    // population[i].fitness = calculateFitness(population[i]);
  }
  // 等待线程全部结束
  // std::cout << "yzx" << std::endl;
  for (auto it = threads.begin(); it != threads.end(); ++it) {
    it->join();
  }
  // std::cout << "yzx" << std::endl;
}

// 淘汰个体
void discard(std::vector<Individual> &population) {
  // 淘汰个体
  std::sort(population.begin(), population.end(),
            [](const Individual &a, const Individual &b) {
              return a.fitness > b.fitness;
            });
  for (int i = 0; i < DISCARD_NUM; ++i) {
    population.pop_back();
  }
}

// 选择操作（轮盘赌选择）
Individual selectParent(std::vector<Individual> &population) {
  int totalFitness = 0;
  for (const auto &individual : population) {
    // if (individual.fitness > totalFitness){
    totalFitness += individual.fitness;
    // }
  }

  int randomFitness = getRandomInt(0, totalFitness);
  int cumulativeFitness = 0;
  for (const auto &individual : population) {
    cumulativeFitness += individual.fitness;
    if (cumulativeFitness >= randomFitness) {
      // std::cout << "不是第一个" << std::endl;
      return individual;
    }
  }

  return population[0];  // 如果未能选择到个体，则返回第一个个体
}

// 交叉操作
Individual crossover(const Individual &parent1, const Individual &parent2) {
  Individual child;

  // 单点交叉
  // int crossoverPoint = getRandomInt(1, NUM_PARAMETERS - 1);
  for (int i = 0; i < NUM_PARAMETERS; i++) {
    if (static_cast<double>(rand()) / RAND_MAX < 0.5) {
      child.genes[i] = parent1.genes[i];
    } else {
      child.genes[i] = parent2.genes[i];
    }
  }

  return child;
}

// 变异操作
void mutate(Individual &individual) {
  for (int i = 0; i < NUM_PARAMETERS; ++i) {
    if (static_cast<double>(rand()) / RAND_MAX < MUTATION_RATE) {
      std::cout << "变异" << std::endl;
      individual.genes[i] =
          getRandomInt(individual.min_value[i], individual.max_value[i]);
    }
  }
}

std::map<std::string, int> res;

// 遗传算法主循环
void geneticAlgorithm(std::vector<Individual> &population) {
  for (int generation = 0; generation < MAX_GENERATIONS; ++generation) {
    evaluatePopulation(population);

    // 输出每一代的最佳个体适应度值
    int bestFitness = population[0].fitness;
    int index = -1;  // 记录最好个体的下标
    int count = 0;   // 计数
    for (const auto &individual : population) {
      if (individual.fitness > bestFitness) {
        bestFitness = individual.fitness;
        index = count;
      }
      count++;
    }
    // std::cout << "yzx" << std::endl;
    std::cout << "Generation " << generation
              << ", Best Fitness: " << bestFitness << "个体 " << index
              << std::endl;
    // 把最优的基因留下来
    Individual parent_max;
    if (index == -1) {
      // 表明这一代中没有最优的个体
      std::cout << "第" << generation << "代第一个是最优的个体" << std::endl;
      std::cout << "TOLERANT_TIME = " << population[0].genes[0] << std::endl;
      std::cout << "TOLERANT_LEAVE_TIME = " << population[0].genes[1]
                << std::endl;
      std::cout << "GOODS_VALUE_VALVE = " << population[0].genes[2]
                << std::endl;
      std::cout << "GOODS_FILTER_VALVE_NUM = " << population[0].genes[3]
                << std::endl;
      std::cout << "VALUEABLE_GOODS_VALVE = " << population[0].genes[4]
                << std::endl;
      std::cout << "FINAL_TOLERANT_TIME = " << population[0].genes[5]
                << std::endl;
      std::cout << "BUSY_VALVE = " << population[0].genes[6] << std::endl;
      std::cout << "BOAT_CAPACITY_REDUCE = " << population[0].genes[7]
                << std::endl;
      std::cout << "MAX_ROBOT_NUM_1 = " << population[0].genes[8] << std::endl;
      std::cout << "MAX_ROBOT_NUM_2 = " << population[0].genes[9] << std::endl;
      std::cout << "MAX_BOAT_NUM = " << population[0].genes[10] << std::endl;
      std::cout << "AVR_MONEY_DIFFERENTIAL = " << population[0].genes[11] << std::endl;
      std::cout << "FIND_NEIGHBOR_MAX_ROBOT = " << population[0].genes[12] << std::endl;
      parent_max = population[0];
      // 最后一代存入res
      if (generation == MAX_GENERATIONS - 1) {
        res["TOLERANT_TIME"] = population[0].genes[0];
        res["TOLERANT_LEAVE_TIME"] = population[0].genes[1];
        res["GOODS_VALUE_VALVE"] = population[0].genes[2];
        res["GOODS_FILTER_VALVE_NUM"] = population[0].genes[3];
        res["VALUEABLE_GOODS_VALVE"] = population[0].genes[4];
        res["FINAL_TOLERANT_TIME"] = population[0].genes[5];
        res["BUSY_VALVE"] = population[0].genes[6];
        res["BOAT_CAPACITY_REDUCE"] = population[0].genes[7];
        res["MAX_ROBOT_NUM_1"] = population[0].genes[8];
        res["MAX_ROBOT_NUM_2"] = population[0].genes[9];
        res["MAX_BOAT_NUM"] = population[0].genes[10];
        res["AVR_MONEY_DIFFERENTIAL"] = population[0].genes[11];
        res["FIND_NEIGHBOR_MAX_ROBOT"] = population[0].genes[12];
      }
    } else {
      std::cout << "第" << generation << "代最优的个体" << std::endl;
      // for (auto it = population[index].genes.begin(); it !=
      // population[index].genes.end(); ++it){
      //   std::cout << *it << " ";
      // }
      std::cout << "TOLERANT_TIME = " << population[index].genes[0]
                << std::endl;
      std::cout << "TOLERANT_LEAVE_TIME = " << population[index].genes[1]
                << std::endl;
      std::cout << "GOODS_VALUE_VALVE = " << population[index].genes[2]
                << std::endl;
      std::cout << "GOODS_FILTER_VALVE_NUM = " << population[index].genes[3]
                << std::endl;
      std::cout << "VALUEABLE_GOODS_VALVE = " << population[index].genes[4]
                << std::endl;
      std::cout << "FINAL_TOLERANT_TIME = " << population[index].genes[5]
                << std::endl;
      std::cout << "BUSY_VALVE = " << population[index].genes[6] << std::endl;
      std::cout << "BOAT_CAPACITY_REDUCE = " << population[index].genes[7]
                << std::endl;
      std::cout << "MAX_ROBOT_NUM_1 = " << population[index].genes[8]
                << std::endl;
      std::cout << "MAX_ROBOT_NUM_2 = " << population[index].genes[9] << std::endl;
      std::cout << "MAX_BOAT_NUM = " << population[index].genes[10] << std::endl;
      std::cout << "AVR_MONEY_DIFFERENTIAL = " << population[index].genes[11] << std::endl;
      std::cout << "FIND_NEIGHBOR_MAX_ROBOT = " << population[index].genes[12] << std::endl;
      parent_max = population[index];
      // 最后一代存入res
      if (generation == MAX_GENERATIONS - 1) {
        res["TOLERANT_TIME"] = population[index].genes[0];
        res["TOLERANT_LEAVE_TIME"] = population[index].genes[1];
        res["GOODS_VALUE_VALVE"] = population[index].genes[2];
        res["GOODS_FILTER_VALVE_NUM"] = population[index].genes[3];
        res["VALUEABLE_GOODS_VALVE"] = population[index].genes[4];
        res["FINAL_TOLERANT_TIME"] = population[index].genes[5];
        res["BUSY_VALVE"] = population[index].genes[6];
        res["BOAT_CAPACITY_REDUCE"] = population[index].genes[7];
        res["MAX_ROBOT_NUM_1"] = population[index].genes[8];
        res["MAX_ROBOT_NUM_2"] = population[index].genes[9];
        res["MAX_BOAT_NUM"] = population[index].genes[10];
        res["AVR_MONEY_DIFFERENTIAL"] = population[index].genes[11];
        res["FIND_NEIGHBOR_MAX_ROBOT"] = population[index].genes[12];
      }
    }

    std::vector<Individual> newPopulation;
    // 生成新种群
    // 淘汰不合格的个体
    discard(population);
    for (int i = 0; i < POPULATION_SIZE - 1; ++i) {
      Individual parent1 = selectParent(population);
      Individual parent2 = selectParent(population);

      Individual child = crossover(parent1, parent2);

      mutate(child);

      newPopulation.push_back(child);
    }
    newPopulation.push_back(parent_max);

    population = newPopulation;
  }
}

int gaRun(std::map<std::string, int> param) {
  srand(static_cast<unsigned>(time(0)));

  std::vector<Individual> population;
  // std::cout << "yyy" << std::endl;
  initializePopulation(population, param);
  // std::cout << "yyy" << std::endl;
  geneticAlgorithm(population);
  // std::cout << "yyy" << std::endl;
  std::ofstream outputFile("./gasa/p.txt", std::ios::app);
  if (outputFile.is_open()) {
    outputFile << "遗传算法" << std::endl;
    outputFile << "int tolerant_time_ = " << res["TOLERANT_TIME"] << ";"
               << std::endl;
    outputFile << "int tolerant_leave_time_ = " << res["TOLERANT_LEAVE_TIME"]
               << ";" << std::endl;
    outputFile << "int goods_value_valve_ = " << res["GOODS_VALUE_VALVE"] << ";"
               << std::endl;
    outputFile << "int goods_filter_valve_num_ = "
               << res["GOODS_FILTER_VALVE_NUM"] << ";" << std::endl;
    outputFile << "int valueable_goods_valve_ = "
               << res["VALUEABLE_GOODS_VALVE"] << ";" << std::endl;
    outputFile << "int final_tolerant_time_ = " << res["FINAL_TOLERANT_TIME"]
               << ";" << std::endl;
    outputFile << "int busy_valve_ = " << res["BUSY_VALVE"] << ";" << std::endl;
    outputFile << "int boat_capacity_reduce_ = " << res["BOAT_CAPACITY_REDUCE"]
               << ";" << std::endl;
    outputFile << "int max_robot_num_1_ = " << res["MAX_ROBOT_NUM_1"] << ";"
               << std::endl;
    outputFile << "int max_robot_num_2_ = " << res["MAX_ROBOT_NUM_2"] << ";"
                << std::endl;
    outputFile << "int max_boat_num_ = " << res["MAX_BOAT_NUM"] << ";"
               << std::endl;
    outputFile << "int avr_money_differential_ = " << res["AVR_MONEY_DIFFERENTIAL"] << ";"
                << std::endl;
    outputFile << "int find_neighbor_max_robot_ = " << res["FIND_NEIGHBOR_MAX_ROBOT"] << ";"
                << std::endl;
    outputFile.close();  // 关闭文件
    std::cout << "写入money成功" << std::endl;
  } else {
    std::cerr << "无法打开文件进行写入操作。" << std::endl;
  }
  return 0;
}