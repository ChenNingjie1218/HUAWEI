#ifndef PARAM_H_
#define PARAM_H_
// #define DEBUG

const int k_Cost1 = 0;
const int k_Cost2 = 2;
#define COL 210
#define ROW 210
// 货物生存周期
const int LIFETIME = 1000;

// 地图长度
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

// 船换泊位耗时
#define CHANGE_BERTH_TIME 500

// 决策类型
#define DECISION_TYPE_ROBOT_MOVE 1
#define DECISION_TYPE_ROBOT_GET 2
#define DECISION_TYPE_ROBOT_PULL 3
#define DECISION_TYPE_BOAT_SHIP 4
#define DECISION_TYPE_BOAT_GO 5
#define DECISION_ROBOT_RIGHT 0
#define DECISION_ROBOT_LEFT 1
#define DECISION_ROBOT_UP 2
#define DECISION_ROBOT_DOWN 3

#define BERTH_WEIGHT_AFTER_BOAT_CHOOSE 1  //船选择泊位后，泊位权重的减少

// A*算法深度
#define CUT_A_STAR
#define DEFAULT_A_STAR_DEEP 150

// 机器人找货物容忍步数
#define TOLERANT_TIME 30

// 船最后走可容忍时间的
#define TOLERANT_LEAVE_TIME 10

// 船在港口停留时间
#define TOLERANT_WAIT_TIME 1000

// 货物筛选域值
#define GOODS_VALUE_VALVE 20

// 贵重货物域值
#define VALUEABLE_GOODS_VALVE 150

// 最后冲刺阶段的容忍时间
#define FINAL_TOLERANT_TIME 100
#endif
