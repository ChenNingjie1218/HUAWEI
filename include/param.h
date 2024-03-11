#ifndef PARAM_H_
#define PARAM_H_
const int k_Cost1 = 10;  //走一格消耗10
const int k_Cost2 = 14;  //斜移走一个消耗14
#define COL 210
#define ROW 210
// 货物生存周期
const int LIFETIME = 1000;

// 地图长度
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

// 决策类型
#define DECISION_TYPE_ROBOT_MOVE 1
#define DECISION_TYPE_ROBOT_GET 2
#define DECISION_TYPE_ROBOT_PULL 3
#define DECISION_TYPE_BOAT_SHIP 4
#define DECISION_TYPE_BOAT_GO 5
#define DECISION_ROBOT_RIGHT 1
#define DECISION_ROBOT_LEFT 2
#define DECISION_ROBOT_UP 3
#define DECISION_ROBOT_DOWN 4

#define BERTH_WEIGHT_AFTER_BOAT_CHOOSE 1  //船选择泊位后，泊位权重的减少
#endif
