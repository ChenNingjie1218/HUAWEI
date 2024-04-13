
#include "map_controller.h"

#include <iostream>

#include "rent_controller.h"

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
      land_area[id] = FindArea(land_area[id], is_land);
    }
    return land_area[id];
  } else {
    if (sea_area[id] != id) {
      sea_area[id] = FindArea(sea_area[id], is_land);
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
  Location initial(berth_x, berth_y);
  location_to_berth_id[initial] = berth_id;
  berth[berth_id].loc.push_back(initial);
  q.push(initial);
  while (!q.empty()) {
    Location temp = q.front();
    q.pop();
    for (int i = 0; i < 4; ++i) {
      int x = temp.x + DIRS[i].x;
      int y = temp.y + DIRS[i].y;
      Location new_point(x, y);
      if (x > 0 && x <= n && y > 0 && y <= n &&
          location_to_berth_id.find(new_point) == location_to_berth_id.end() &&
          (ch[x][y] == 'B' || ch[x][y] == 'K')) {
        location_to_berth_id[new_point] = berth_id;
        if (ch[x][y] == 'B') {
          berth[berth_id].loc.push_back(new_point);
        }
        q.push(new_point);
      }
    }
  }
#ifdef DEBUG
  int size = berth[berth_id].loc.size();
  std::cerr << "泊位 " << berth_id << " 坐标：" << std::endl;
  for (int i = 0; i < size; ++i) {
    Location& loc = berth[berth_id].loc[i];
    std::cerr << "(" << loc.x << "," << loc.y << ")" << std::endl;
  }
  // for (auto it = location_to_berth_id.begin(); it !=
  // location_to_berth_id.end();
  //      ++it) {
  //   std::cerr << "泊位：" << it->second << " 坐标："
  //             << "(" << it->first.x << "," << it->first.y << ")" <<
  //             std::endl;
  // }
#endif
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

      // 初始化最近机器人购买点id
      nearest_r[i][j] = -1;

      // 初始化最近船购买点id
      nearest_s[i][j] = -1;
#ifdef TEST_ASTAR
      astar_debug[i][j] = 0;
#endif
    }
  }
#ifdef FACE_MAP
  InitMap1Tag();
  InitMap2Tag();
  if (delivery_point == map1_tag) {
#ifdef DEBUG
    std::cerr << "map1" << std::endl;
#endif
    InitMapParam(1);
    InitDashTable(1);
  } else if (delivery_point == map2_tag) {
#ifdef DEBUG
    std::cerr << "map2" << std::endl;
#endif
    InitMapParam(2);
    InitDashTable(2);
  } else {
#ifdef DEBUG
    std::cerr << "map3" << std::endl;
#endif
    InitMapParam(3);
    InitDashTable(3);
  }
#endif
  // 初始化nearest_delivery
  InitNearestDelivery();
  // 初始化nearest_s
  InitNearestS();
  // 初始化nearest_r
  InitNearestR();

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
  int berth_size = berth.size();
  for (int i = 0; i < berth_size; i++) {
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

// 初始化nearest_r
void MapController::InitNearestR() {
  std::queue<std::pair<Location, int>> q;
  int size = robot_purchase_point.size();
  for (int i = 0; i < size; ++i) {
    q.push(std::make_pair(robot_purchase_point[i], i));
  }
  while (!q.empty()) {
    std::pair<Location, int> temp = q.front();
    q.pop();
    for (int i = 0; i < 4; ++i) {
      int x = temp.first.x + DIRS[i].x;
      int y = temp.first.y + DIRS[i].y;
      if (x > 0 && x <= n && y > 0 && y <= n) {
        if (nearest_r[x][y] == -1 && CanRobotReach(x, y)) {
          nearest_r[x][y] = temp.second;
          q.push(std::make_pair(Location(x, y), temp.second));
        }
      }
    }
  }
}

// 初始化nearest_s
void MapController::InitNearestS() {
  std::queue<std::pair<Location, int>> q;
  int size = boat_purchase_point.size();
  for (int i = 0; i < size; ++i) {
    q.push(std::make_pair(boat_purchase_point[i], i));
  }
  while (!q.empty()) {
    std::pair<Location, int> temp = q.front();
    q.pop();
    for (int i = 0; i < 4; ++i) {
      int x = temp.first.x + DIRS[i].x;
      int y = temp.first.y + DIRS[i].y;
      if (x > 0 && x <= n && y > 0 && y <= n) {
        if (nearest_s[x][y] == -1 && CanBoatReach(x, y)) {
          nearest_s[x][y] = temp.second;
          q.push(std::make_pair(Location(x, y), temp.second));
        }
      }
    }
  }
}

// 填地图
void MapController::FillMap(int x, int y) {
  std::multimap<int, int> openlist;
  std::multimap<int, int> closeList;
  openlist.insert(std::pair<int, int>(x, y));
  while (!openlist.empty()) {
    auto p = openlist.begin();
    auto point = *p;
    openlist.erase(p);
    closeList.insert(point);
    // 获取point周围的四个点
    int dx[4] = {0, 0, 1, -1};
    int dy[4] = {1, -1, 0, 0};
    for (int i = 0; i < 4; i++) {
      int new_x = point.first + dx[i];
      int new_y = point.second + dy[i];
      if (new_x < 1 || new_x > n || new_y < 1 || new_y > n) {
        continue;
      }
      if (ch[new_x][new_y] == 'K' || ch[new_x][new_y] == 'B') {
        // 判断这个新的点是不是在closeList里面
        bool flag = false;
        for (auto it = closeList.begin(); it != closeList.end(); ++it) {
          if (it->first == new_x && it->second == new_y) {
            flag = true;
            break;
          }
        }
        if (flag) {
          continue;
        }
        openlist.insert(std::pair<int, int>(new_x, new_y));
      }
    }
    // 把当前点填充
    ch[point.first][point.second] = '~';
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
#ifdef FACE_MAP
// 面向地图调参
void MapController::InitMap1Tag() {
  map1_tag.push_back(Location(198, 11));
  map1_tag.push_back(Location(198, 192));
}
void MapController::InitMap2Tag() {
  map2_tag.push_back(Location(3, 198));
  map2_tag.push_back(Location(198, 198));
}

void MapController::InitMapParam(int id) {
  auto& param_instance = DynamicParam::GetInstance();
  switch (id) {
    case 1:
      map1 = true;
      break;
    case 2:
      map2 = true;
      break;
    case 3:
      break;
  }
}

// 初始化冲刺表
void MapController::InitDashTable(int map_id) {
  switch (map_id) {
    case 1:
      dash_table.push_back(std::make_pair(15001, 4));  // 0
      dash_table.push_back(std::make_pair(15001, 4));  // 1
      dash_table.push_back(std::make_pair(15001, 4));  // 2
      dash_table.push_back(std::make_pair(15001, 4));  // 3
      dash_table.push_back(std::make_pair(15001, 4));  // 4
      dash_table.push_back(std::make_pair(15001, 4));  // 5
      dash_table.push_back(std::make_pair(15001, 4));  // 6
      dash_table.push_back(std::make_pair(15001, 4));  // 7
      dash_table.push_back(std::make_pair(15001, 4));  // 8
      dash_table.push_back(std::make_pair(15001, 4));  // 9
      dash_table.push_back(std::make_pair(15001, 4));  // 10
      dash_table.push_back(std::make_pair(15001, 4));  // 11
      dash_table.push_back(std::make_pair(15001, 4));  // 12
      dash_table.push_back(std::make_pair(15001, 4));  // 13
      dash_table.push_back(std::make_pair(15001, 4));  // 14
      dash_table.push_back(std::make_pair(15001, 4));  // 15
      dash_table.push_back(std::make_pair(15001, 4));  // 16
      dash_table.push_back(std::make_pair(15001, 4));  // 17
      dash_table.push_back(std::make_pair(15001, 4));  // 18

      break;
    case 2:
      dash_table.push_back(std::make_pair(15001, 4));  // 0
      dash_table.push_back(std::make_pair(15001, 4));  // 1
      dash_table.push_back(std::make_pair(15001, 4));  // 2
      dash_table.push_back(std::make_pair(15001, 4));  // 3
      dash_table.push_back(std::make_pair(15001, 4));  // 4
      dash_table.push_back(std::make_pair(15001, 4));  // 5
      dash_table.push_back(std::make_pair(15001, 4));  // 6
      dash_table.push_back(std::make_pair(15001, 4));  // 7
      dash_table.push_back(std::make_pair(15001, 4));  // 8
      dash_table.push_back(std::make_pair(15001, 4));  // 9
      dash_table.push_back(std::make_pair(15001, 4));  // 10
      dash_table.push_back(std::make_pair(15001, 4));  // 11
      dash_table.push_back(std::make_pair(15001, 4));  // 12
      dash_table.push_back(std::make_pair(15001, 4));  // 13
      dash_table.push_back(std::make_pair(15001, 4));  // 14
      dash_table.push_back(std::make_pair(15001, 4));  // 15
      dash_table.push_back(std::make_pair(15001, 4));  // 16
      dash_table.push_back(std::make_pair(15001, 4));  // 17
      dash_table.push_back(std::make_pair(15001, 4));  // 18
      break;
    case 3:
      dash_table.push_back(std::make_pair(15001, 4));  // 0
      dash_table.push_back(std::make_pair(15001, 4));  // 1
      dash_table.push_back(std::make_pair(15001, 4));  // 2
      dash_table.push_back(std::make_pair(15001, 4));  // 3
      dash_table.push_back(std::make_pair(15001, 4));  // 4
      dash_table.push_back(std::make_pair(15001, 4));  // 5
      dash_table.push_back(std::make_pair(15001, 4));  // 6
      dash_table.push_back(std::make_pair(15001, 4));  // 7
      dash_table.push_back(std::make_pair(15001, 4));  // 8
      dash_table.push_back(std::make_pair(15001, 4));  // 9
      dash_table.push_back(std::make_pair(15001, 4));  // 10
      dash_table.push_back(std::make_pair(15001, 4));  // 11
      dash_table.push_back(std::make_pair(15001, 4));  // 12
      dash_table.push_back(std::make_pair(15001, 4));  // 13
      dash_table.push_back(std::make_pair(15001, 4));  // 14
      dash_table.push_back(std::make_pair(15001, 4));  // 15
      dash_table.push_back(std::make_pair(15001, 4));  // 16
      dash_table.push_back(std::make_pair(15001, 4));  // 17
      dash_table.push_back(std::make_pair(15001, 4));  // 18
      break;
  }
}

#endif