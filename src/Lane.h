#ifndef PATH_PLANNING_LANE_H
#define PATH_PLANNING_LANE_H

class Lane {
public:
  explicit Lane(int laneNumber);

  int lane;
  bool isAvailable;
  double nearestForwardEnemyDist;
  double nearestForwardEnemyVelocity;

  double Cost();
};

#endif //PATH_PLANNING_LANE_H
