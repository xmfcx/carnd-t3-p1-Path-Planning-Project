#ifndef PATH_PLANNING_LANEHANDLER_H
#define PATH_PLANNING_LANEHANDLER_H

#include <vector>
#include "Lane.h"

class LaneHandler {
public:
  std::vector<Lane> lanes;

  Lane &getLane(int laneNumber);

  int getLowestCostLane(int lane_car);
};

#endif //PATH_PLANNING_LANEHANDLER_H
