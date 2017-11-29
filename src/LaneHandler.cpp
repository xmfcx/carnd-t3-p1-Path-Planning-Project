#include <cmath>
#include "LaneHandler.h"

Lane &LaneHandler::getLane(int laneNumber) {
  for (int i = 0; i < 3; i++) {
    if (lanes[i].lane == laneNumber) {
      return lanes[i];
    }
  }
  return lanes[0];
}

int LaneHandler::getLowestCostLane(int lane_car) {
  int index_min = 99;
  double cost_min = 1;
  for (int i = 0; i < 3; i++) {
    if (!lanes[i].isAvailable)
      continue;
    int lane_current = lanes[i].lane;

    // We only check left, current, right lanes
    if (std::abs(lane_car - lane_current) > 1)
      continue;

    // Assign lowest cost&index if it is
    double cost_current = lanes[i].Cost();
    if (cost_current < cost_min) {
      cost_min = cost_current;
      index_min = i;
    }
  }
  if (index_min == 99)
    return lane_car;
  return lanes[index_min].lane;
}