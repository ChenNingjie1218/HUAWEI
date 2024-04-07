#include "map_controller.h"

#include <iostream>

MapController* MapController::instance_ = nullptr;
MapController*& MapController::GetInstance() {
  if (!instance_) {
    instance_ = new MapController();
  }
  return instance_;
}
// 用并查集分区
// 找集合
int MapController::FindArea(int id, bool is_land) {
  if (is_land) {
    if (land_area[id] != id) {
      land_area[id] = FindArea(land_area[id]);
    }
    return land_area[id];
  } else {
    if (sea_area[id] != id) {
      sea_area[id] = FindArea(sea_area[id]);
    }
    return sea_area[id];
  }
}
// 合并集合
void MapController::MergeArea(int id_1, int id_2, bool is_land) {
  int root_1 = FindArea(id_1, is_land);
  int root_2 = FindArea(id_2, is_land);
  if (root_1 != root_2) {
    if (is_land) {
      land_area[root_2] = root_1;
    } else {
      sea_area[root_2] = root_1;
    }
  }
}

// 机器人可达
bool MapController::CanRobotReach(int x, int y) {
  return ch[x][y] == '>' || ch[x][y] == 'R' || ch[x][y] == 'B' ||
         ch[x][y] == 'C' || ch[x][y] == 'c' || ch[x][y] == '.';
}

// 船可达
bool MapController::CanBoatReach(int x, int y) {
  return ch[x][y] == '~' || ch[x][y] == '*' || ch[x][y] == 'S' ||
         ch[x][y] == 'B' || ch[x][y] == 'K' || ch[x][y] == 'C' ||
         ch[x][y] == 'c' || ch[x][y] == 'T';
}

// 初始化坐标映射到泊位id的map
void MapController::InitBerthMap(int berth_id, int berth_x, int berth_y) {
  std::queue<Location> q;
  q.push(Location(berth_x, berth_y));
  while (!q.empty()) {
    Location temp = q.front();
    q.pop();
    location_to_berth_id[temp] = berth_id;
    for (int i = 0; i < 4; ++i) {
      int x = temp.x + DIRS[i].x;
      int y = temp.y + DIRS[i].y;
      Location new_point(x, y);
      if (x > 0 && x <= n && y > 0 && y <= n &&
          location_to_berth_id.find(new_point) == location_to_berth_id.end() &&
          (ch[x][y] == 'B' || ch[x][y] == 'K')) {
        q.push(new_point);
      }
    }
  }
}

// 初始化各项数据
void MapController::InitMapData() {
  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= n; ++j) {
      // 记录购买点、交货点
      if (ch[i][j] == 'R') {
        robot_purchase_point.push_back(Location(i, j));
      } else if (ch[i][j] == 'S') {
        boat_purchase_point.push_back(Location(i, j));
      } else if (ch[i][j] == 'T') {
        delivery_point.push_back(Location(i, j));
      }
      // 初始化堵车标记
      busy_point[i][j] = 0;

      // 初始化区域号
      land_area[i * n + j] = i * n + j;
      sea_area[i * n + j] = i * n + j;

      // 初始化最近泊位id
      nearest_berth[i][j] = -1;

      // 初始化最近交货点id
      nearest_delivery[i][j] = -1;
#ifdef TEST_ASTAR
      astar_debug[i][j] = 0;
#endif
    }
  }

  // 初始化nearest_delivery
  InitNearestDelivery();

  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= n; ++j) {
      if (CanRobotReach(i, j)) {  // 陆地区域分区
        if (CanRobotReach(i - 1, j)) {
          MergeArea(i * n + j, (i - 1) * n + j);
        }
        if (CanRobotReach(i, j - 1)) {
          MergeArea(i * n + j, i * n + j - 1);
        }
      }
      if (CanBoatReach(i, j)) {  // 船区域分区
        if (CanBoatReach(i - 1, j)) {
          MergeArea(i * n + j, (i - 1) * n + j, false);
        }
        if (CanBoatReach(i, j - 1)) {
          MergeArea(i * n + j, i * n + j - 1, false);
        }
      }
    }
  }
}

// 初始化nearest_berth
void MapController::InitNearestBerth(std::queue<std::pair<Location, int>>& q) {
  while (!q.empty()) {
    std::pair<Location, int> temp = q.front();
    q.pop();
    for (int i = 0; i < 4; ++i) {
      int x = temp.first.x + DIRS[i].x;
      int y = temp.first.y + DIRS[i].y;
      if (x > 0 && x <= n && y > 0 && y <= n) {
        if (nearest_berth[x][y] == -1 && CanRobotReach(x, y)) {
          nearest_berth[x][y] = temp.second;
          q.push(std::make_pair(Location(x, y), temp.second));
        } else if (nearest_berth[x][y] != -1 &&
                   nearest_berth[x][y] != temp.second) {
          // 加入邻居
          berth[nearest_berth[x][y]].neighbor.insert(temp.second);
          berth[temp.second].neighbor.insert(nearest_berth[x][y]);
        }
      }
    }
  }

#ifdef DEBUG
  for (int i = 0; i < berth.size(); i++) {
    std::cerr << i << "neighbor:" << std::endl;

    for (std::set<int>::iterator it = berth[i].neighbor.begin();
         it != berth[i].neighbor.end(); ++it) {
      std::cerr << *it << ' ';
    }
    std::cerr << "size:" << berth[i].neighbor.size() << std::endl;
  }
#endif
}

// 初始化nearest_delivery
void MapController::InitNearestDelivery() {
  std::queue<std::pair<Location, int>> q;
  int size = delivery_point.size();
  for (int i = 0; i < size; ++i) {
    q.push(std::make_pair(delivery_point[i], i));
  }
  while (!q.empty()) {
    std::pair<Location, int> temp = q.front();
    q.pop();
    for (int i = 0; i < 4; ++i) {
      int x = temp.first.x + DIRS[i].x;
      int y = temp.first.y + DIRS[i].y;
      if (x > 0 && x <= n && y > 0 && y <= n) {
        if (nearest_delivery[x][y] == -1 && CanBoatReach(x, y)) {
          nearest_delivery[x][y] = temp.second;
          q.push(std::make_pair(Location(x, y), temp.second));
        }
      }
    }
  }
}

// 是否是主干道
bool MapController::IsMainRoad(int x, int y) {
  return ch[x][y] == '>' || ch[x][y] == 'c' || ch[x][y] == 'R';
}

// 是否是主航道
bool MapController::IsMainChannel(int x, int y) {
  return ch[x][y] == '~' || ch[x][y] == 'c' || ch[x][y] == 'T' ||
         ch[x][y] == 'K' || ch[x][y] == 'S';
}
