#include <cstdio>
#include <iostream>

#include "astar.h"
#include "map_controller.h"

// 方向数组
extern std::array<Location, 4> DIRS;

int main() {
  char(&ch)[N][N] = MapController::GetInstance()->ch;
  FILE* fp = fopen("../../doc/map1.txt", "r");
  // 地图数据
  for (int i = 1; i <= n; i++) fscanf(fp, "%s", ch[i] + 1);
  fclose(fp);
  MapController::GetInstance()->InitMapData();
  std::cerr << "地图初始化完毕！" << std::endl;
  Astar astar(50, 121, 198, 4, BOAT_DIRECTION_RIGHT);
  std::vector<int> path;
  astar.AstarSearch(path);
  int size = path.size();
  Location cur(50, 121, BOAT_DIRECTION_RIGHT);
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

  fp = fopen("../../debug/astar_map.txt", "w");
  // 地图数据
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= n; ++j) {
      fprintf(fp, "%c", ch[i][j]);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return 0;
}