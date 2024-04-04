#include <chrono>
#include <cstdio>
#include <iostream>

#include "astar.h"
#include "map_controller.h"
#define ROBOT_GOODS  // 机器人找货物
// #define ROBOT_BERTH  // 机器人找泊位
#define BOAT  // 船
// 方向数组
extern std::array<Location, 4> DIRS;
#ifdef DEBUG
FILE *debug_map_file = fopen("./debug/debug_map.txt", "w");
FILE *debug_command_file = fopen("./debug/debug.txt", "w");
FILE *debug_output_file = fopen("./debug/cerr.txt", "w");
#endif
int main() {
  char(&ch)[N][N] = MapController::GetInstance()->ch;
  FILE *fp = fopen("../../doc/map2.txt", "r");
  // 地图数据
  for (int i = 1; i <= n; i++) fscanf(fp, "%s", ch[i] + 1);
  fclose(fp);
  MapController::GetInstance()->InitMapData();
  std::cerr << "地图初始化完毕！" << std::endl;
#ifdef ROBOT_BERTH
  Astar astar(110, 144, 100, 17);
  MapController::GetInstance()->InitBerthMap(0, 100, 17);
  // MapController::GetInstance()->InitBerthMap(1, 103, 92);
  std::vector<Location> path;
  // 获取当前时间点
  auto start = std::chrono::high_resolution_clock::now();
  astar.AstarSearch(path, 0);
  auto end = std::chrono::high_resolution_clock::now();
  // 计算执行时间（以毫秒为单位）
  std::chrono::duration<double, std::milli> duration = end - start;
  std::cerr << "耗时：" << duration.count() << " ms" << std::endl;
  int size = path.size();
  for (int i = 0; i < size; ++i) {
    int x = path[i].x;
    int y = path[i].y;
    ch[x][y] = 'O';
  }
#endif

#ifdef BOAT
  // Astar astar(3, 197, 168, 100, BOAT_DIRECTION_DOWN); // map1的交货点到某泊位
  Astar astar(103, 92, 198, 196, BOAT_DIRECTION_RIGHT);  // map2
  std::vector<int> path;
  // 获取当前时间点
  auto start = std::chrono::high_resolution_clock::now();
  astar.AstarSearch(path);
  auto end = std::chrono::high_resolution_clock::now();
  // 计算执行时间（以毫秒为单位）
  std::chrono::duration<double, std::milli> duration = end - start;
  std::cerr << "耗时：" << duration.count() << " ms" << std::endl;
  int size = path.size();
  // Location cur(3, 197, BOAT_DIRECTION_DOWN); // map1的交货点到某泊位
  Location cur(103, 92, BOAT_DIRECTION_RIGHT);
  char ship[4] = {'>', '<', '^', 'V'};
  for (int i = 0; i < size; ++i) {
    ch[cur.x][cur.y] = ship[cur.boat_direction];
    if (cur.boat_direction == path[i]) {
      cur = Location(cur.x + DIRS[cur.boat_direction].x,
                     cur.y + DIRS[cur.boat_direction].y, path[i]);
    } else {
      switch (cur.boat_direction) {
        case BOAT_DIRECTION_RIGHT:
          if (path[i] == BOAT_DIRECTION_UP) {
            cur = cur.CounterClockwise();
          } else if (path[i] == BOAT_DIRECTION_DOWN) {
            cur = cur.Clockwise();
          } else {
            std::cerr << "方向错了" << std::endl;
          }
          break;
        case BOAT_DIRECTION_LEFT:
          if (path[i] == BOAT_DIRECTION_DOWN) {
            cur = cur.CounterClockwise();
          } else if (path[i] == BOAT_DIRECTION_UP) {
            cur = cur.Clockwise();
          } else {
            std::cerr << "方向错了" << std::endl;
          }
          break;
        case BOAT_DIRECTION_UP:
          if (path[i] == BOAT_DIRECTION_LEFT) {
            cur = cur.CounterClockwise();
          } else if (path[i] == BOAT_DIRECTION_RIGHT) {
            cur = cur.Clockwise();
          } else {
            std::cerr << "方向错了" << std::endl;
          }
          break;
        case BOAT_DIRECTION_DOWN:
          if (path[i] == BOAT_DIRECTION_RIGHT) {
            cur = cur.CounterClockwise();
          } else if (path[i] == BOAT_DIRECTION_LEFT) {
            cur = cur.Clockwise();
          } else {
            std::cerr << "方向错了" << std::endl;
          }
          break;
      }
    }
  }
#endif

  fp = fopen("../../debug/astar_map.txt", "w");
  // 地图数据
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= n; ++j) {
      fprintf(fp, "%c", ch[i][j]);
      // fprintf(fp, "%d", MapController::GetInstance()->astar_debug[i][j]);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return 0;
}