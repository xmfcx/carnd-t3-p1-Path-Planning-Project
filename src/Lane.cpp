#include <iostream>
#include "Lane.h"

Lane::Lane(int laneNumber) {
  lane = laneNumber;
  isAvailable = true;
  nearestForwardEnemyDist = 1000;
  nearestForwardEnemyVelocity = 50;
}

double Lane::Cost() {
//  std::cout << "lane #" << lane << " nearestForwardEnemyDist= "
//            << nearestForwardEnemyDist << ", nearestForwardEnemyVelocity= " <<
//            nearestForwardEnemyVelocity << std::endl;

  double normalized_dist = nearestForwardEnemyDist / 1000;
  double normalized_vel = nearestForwardEnemyVelocity / 100;

  double cost_dist = 1 - normalized_dist;
  double cost_vel = 1 - normalized_vel;

  double weight_dist = 0.9;
  double weight_vel = 1 - weight_dist;

  double cost_final = cost_dist * weight_dist + cost_vel * weight_vel;

  return cost_final;
}