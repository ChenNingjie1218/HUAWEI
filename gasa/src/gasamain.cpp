#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <random>
#include <tuple>

#include "gasa.h"
int TOLERANT_TIME = 0;
int TOLERANT_LEAVE_TIME = 0;
int GOODS_VALUE_VALVE = 0;
int GOODS_FILTER_VALVE_NUM = 0;
int VALUEABLE_GOODS_VALVE = 0;
int FINAL_TOLERANT_TIME = 0;
int BUSY_VALVE = 0;
int BOAT_CAPACITY_REDUCE = 0;
int MAX_ROBOT_NUM = 1;
int MAX_BOAT_NUM = 1;
std::string WHICHMAP = "map3.txt";
// 目标函数
int objective_function(std::map<std::string, int> paramMap) {
  TOLERANT_TIME = paramMap["TOLERANT_TIME"];
  TOLERANT_LEAVE_TIME = paramMap["TOLERANT_LEAVE_TIME"];
  GOODS_VALUE_VALVE = paramMap["GOODS_VALUE_VALVE"];
  GOODS_FILTER_VALVE_NUM = paramMap["GOODS_FILTER_VALVE_NUM"];
  VALUEABLE_GOODS_VALVE = paramMap["VALUEABLE_GOODS_VALVE"];
  FINAL_TOLERANT_TIME = paramMap["FINAL_TOLERANT_TIME"];
  BUSY_VALVE = paramMap["BUSY_VALVE"];
  BOAT_CAPACITY_REDUCE = paramMap["BOAT_CAPACITY_REDUCE"];
  MAX_ROBOT_NUM = paramMap["MAX_ROBOT_NUM"];
  MAX_BOAT_NUM = paramMap["MAX_BOAT_NUM"];
  // 写入数据到文件
  std::ofstream outputFile(
      "./gasa/data.txt", std::ios::out | std::ios::trunc);  // 打开文件用于写入
  if (outputFile.is_open()) {
    outputFile << TOLERANT_TIME << std::endl;
    outputFile << TOLERANT_LEAVE_TIME << std::endl;
    outputFile << GOODS_VALUE_VALVE << std::endl;
    outputFile << GOODS_FILTER_VALVE_NUM << std::endl;
    outputFile << VALUEABLE_GOODS_VALVE << std::endl;
    outputFile << FINAL_TOLERANT_TIME << std::endl;
    outputFile << BUSY_VALVE << std::endl;
    outputFile << BOAT_CAPACITY_REDUCE << std::endl;
    outputFile << MAX_ROBOT_NUM << std::endl;
    outputFile << MAX_BOAT_NUM << std::endl;
    outputFile << "1" << std::endl;
    outputFile.close();  // 关闭文件
    std::cout << "退火算法写入data成功" << std::endl;
  } else {
    std::cout << "无法打开文件进行写入操作。" << std::endl;
  }
  // const char* command =
  //     "/home/yzx/vscode/workspace/main/HUAWEI/run_simple_demo.sh"; //
  //     ./PreliminaryJudge ./build/src/main -m ./doc/map-3.12.txt -l NONE
  std::string command =
      "./SemiFinalJudge './build/gasa/src/mainparam "
      "-1' -m ./doc/" +
      WHICHMAP + " -r 1.rep -l NONE";
  std::system(command.c_str());
  // 从文件中读取数据
  int money1;
  std::ifstream inputFile1("./gasa/money.txt");  // 打开文件用于读取
  if (inputFile1.is_open()) {
    std::string line;
    while (std::getline(inputFile1, line)) {
      std::cout << line << std::endl;
      money1 = std::stod(line);
    }
    inputFile1.close();  // 关闭文件
    std::cout << "退货算法读取money成功" << std::endl;
  } else {
    std::cout << "无法打开文件进行读取操作。" << std::endl;
    exit(-1);
  }

  return money1;
}

// 模拟退火算法
std::map<std::string, int> simulated_annealing(double initial_temperature,
                                               double final_temperature,
                                               double cooling_rate) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::map<std::string, int> param;

  // 初始化解
  std::uniform_int_distribution<int> distribution(0, 50);
  int tolerant_time = distribution(gen);
  param["TOLERANT_TIME"] = tolerant_time;

  std::uniform_int_distribution<int> distribution1(0, 20);
  int tolerant_leave_time = distribution1(gen);
  param["TOLERANT_LEAVE_TIME"] = tolerant_leave_time;

  std::uniform_int_distribution<int> distribution2(0, 200);
  int goods_value_value = distribution2(gen);
  param["GOODS_VALUE_VALVE"] = goods_value_value;

  std::uniform_int_distribution<int> distribution3(0, 100);
  int goods_filter_value_num = distribution3(gen);
  param["GOODS_FILTER_VALVE_NUM"] = goods_filter_value_num;

  std::uniform_int_distribution<int> distribution4(0, 200);
  int valueable_goods_value = distribution4(gen);
  param["VALUEABLE_GOODS_VALVE"] = valueable_goods_value;

  std::uniform_int_distribution<int> distribution5(0, 1000);
  int final_tolerant_time = distribution5(gen);
  param["FINAL_TOLERANT_TIME"] = final_tolerant_time;

  std::uniform_int_distribution<int> distribution6(0, 1000);
  int busy_value = distribution6(gen);
  param["BUSY_VALVE"] = busy_value;

  std::uniform_int_distribution<int> distribution7(0, 50);
  int boat_capacity_reduce = distribution7(gen);
  param["BOAT_CAPACITY_REDUCE"] = boat_capacity_reduce;

  std::uniform_int_distribution<int> distribution8(1, 25);
  int max_robot_num = distribution8(gen);
  param["MAX_ROBOT_NUM"] = max_robot_num;

  std::uniform_int_distribution<int> distribution9(1, 5);
  int max_boat_num = distribution9(gen);
  param["MAX_BOAT_NUM"] = max_boat_num;

  int current_energy = objective_function(param);

  // 初始化温度
  double temperature = initial_temperature;

  while (temperature > final_temperature) {
    std::map<std::string, int> paramtmp;
    // 在当前解的基础上生成新解
    std::uniform_int_distribution<int> Distribution(0, 50);
    int tolerant_time_new = Distribution(gen);
    paramtmp["TOLERANT_TIME"] = tolerant_time_new;

    std::uniform_int_distribution<int> Distribution1(0, 20);
    int tolerant_leave_time_new = Distribution1(gen);
    paramtmp["TOLERANT_LEAVE_TIME"] = tolerant_leave_time_new;

    std::uniform_int_distribution<int> Distribution2(0, 200);
    int goods_value_value_new = Distribution2(gen);
    paramtmp["GOODS_VALUE_VALVE"] = goods_value_value_new;

    std::uniform_int_distribution<int> Distribution3(0, 100);
    int goods_filter_value_num_new = Distribution3(gen);
    paramtmp["GOODS_FILTER_VALVE_NUM"] = goods_filter_value_num_new;

    std::uniform_int_distribution<int> Distribution4(0, 200);
    int valueable_goods_value_new = Distribution4(gen);
    paramtmp["VALUEABLE_GOODS_VALVE"] = valueable_goods_value_new;

    std::uniform_int_distribution<int> Distribution5(0, 1000);
    int final_tolerant_time_new = Distribution5(gen);
    paramtmp["FINAL_TOLERANT_TIME"] = final_tolerant_time_new;

    std::uniform_int_distribution<int> Distribution6(0, 1000);
    int busy_value_new = Distribution6(gen);
    paramtmp["BUSY_VALVE"] = busy_value_new;

    std::uniform_int_distribution<int> Distribution7(0, 50);
    int boat_capacity_reduce_new = Distribution7(gen);
    paramtmp["BOAT_CAPACITY_REDUCE"] = boat_capacity_reduce_new;

    std::uniform_int_distribution<int> Distribution8(1, 25);
    int max_robot_num_new = distribution8(gen);
    paramtmp["MAX_ROBOT_NUM"] = max_robot_num_new;

    std::uniform_int_distribution<int> Distribution9(1, 5);
    int max_boat_num_new = Distribution9(gen);
    paramtmp["MAX_BOAT_NUM"] = max_boat_num_new;

    // 计算新解的能量（目标函数值）
    int new_energy = objective_function(paramtmp);

    // 接受优解的概率
    double acceptance_probability =
        exp((new_energy - current_energy) / temperature);
    std::uniform_real_distribution<double> distribution_random(0.0, 1.0);
    // 根据概率决定是否接受新解
    // distribution8(gen) < acceptance_probability
    if (new_energy > current_energy ||
        (distribution_random(gen) < acceptance_probability)) {
      tolerant_time = tolerant_time_new;
      tolerant_leave_time = tolerant_leave_time_new;
      goods_value_value = goods_value_value_new;
      goods_filter_value_num = goods_filter_value_num_new;
      valueable_goods_value = valueable_goods_value_new;
      final_tolerant_time = final_tolerant_time_new;
      busy_value = busy_value_new;
      boat_capacity_reduce = boat_capacity_reduce_new;
      max_robot_num = max_robot_num_new;
      max_boat_num = max_boat_num_new;
      current_energy = new_energy;
    }

    // 降低温度
    temperature *= cooling_rate;
  }
  std::map<std::string, int> resmap;
  resmap["TOLERANT_TIME"] = tolerant_time;
  resmap["TOLERANT_LEAVE_TIME"] = tolerant_leave_time;
  resmap["GOODS_VALUE_VALVE"] = goods_value_value;
  resmap["GOODS_FILTER_VALVE_NUM"] = goods_filter_value_num;
  resmap["VALUEABLE_GOODS_VALVE"] = valueable_goods_value;
  resmap["FINAL_TOLERANT_TIME"] = final_tolerant_time;
  resmap["BUSY_VALVE"] = busy_value;
  resmap["BOAT_CAPACITY_REDUCE"] = boat_capacity_reduce;
  resmap["MAX_ROBOT_NUM"] = max_robot_num;
  resmap["MAX_BOAT_NUM"] = max_boat_num;
  return resmap;
}
void getCurrentTime() {
  // 获取当前时间
  std::time_t currentTime = std::time(nullptr);

  // 将时间转换为本地时间
  std::tm* localTime = std::localtime(&currentTime);

  // 输出当前时间
  std::cout << "当前系统时间：";
  std::cout << localTime->tm_year + 1900 << "-";
  std::cout << localTime->tm_mon + 1 << "-";
  std::cout << localTime->tm_mday << " ";
  std::cout << localTime->tm_hour << ":";
  std::cout << localTime->tm_min << ":";
  std::cout << localTime->tm_sec << std::endl;
}

int main() {
  double initial_temperature = 100.0;
  double final_temperature = 80;
  double cooling_rate = 0.7;

  // 运行模拟退火算法
  auto res =
      simulated_annealing(initial_temperature, final_temperature, cooling_rate);
  // 输出退火算法得到的参数
  std::cout << "退火算法" << std::endl;
  for (auto it = res.begin(); it != res.end(); ++it) {
    std::cout << it->first << ": " << it->second << std::endl;
  }
  // 把退火算法的结果写入文件
  std::ofstream outputFile("./gasa/p.txt", std::ios::out | std::ios::trunc);
  if (outputFile.is_open()) {
    outputFile << "退火算法" << std::endl;
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
    outputFile << "int max_robot_num_ = " << res["MAX_ROBOT_NUM"] << ";"
               << std::endl;
    outputFile << "int max_boat_num_ = " << res["MAX_BOAT_NUM"] << ";"
               << std::endl;
    outputFile.close();  // 关闭文件
    std::cerr << "写入money成功" << std::endl;
  } else {
    std::cerr << "无法打开文件进行写入操作。" << std::endl;
  }

  // 遗传算法
  gaRun(res);

  // 输出当前时间
  getCurrentTime();

  return 0;
}