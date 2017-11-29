#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "spline.h"
#include "Lane.h"
#include "LaneHandler.h"
#include "StopWatch.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2((map_y - y), (map_x - x));
  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }
  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double>
getFrenet(double x, double y, double theta, const vector<double> &maps_x,
          const vector<double> &maps_y) {
  size_t next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  size_t prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}


int frenetDToLane(double d) {
  if (d >= 0 && d <= 4)
    return 0;
  if (d > 4 && d <= 8)
    return 1;
  if (d > 8 && d <= 12)
    return 2;
  std::cerr << "frenetDToLane Failed." << std::endl;
  return -1;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  double ref_vel = 0.0; //mph
  StopWatch watch;
  watch.Start();
  double lastTime = 0;
  double lastTimeLaneChangeHappened = 0;

  h.onMessage(
    [&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
      &map_waypoints_dx, &map_waypoints_dy, &lane, &ref_vel, &watch, &lastTime,
      &lastTimeLaneChangeHappened](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event

      // Check if data is meaningful
      if (length < 2 || data[0] != '4' || data[1] != '2') {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        return;
      }

      // Data is meaningful
      // Autonomous driving
      auto s = hasData(data);
      if (s.empty())
        return;

      auto j = json::parse(s);
      string event = j[0].get<string>();

      if (event != "telemetry")
        return;

      // j[1] is the data JSON object
      // Main car's localization Data
      double car_x = j[1]["x"];
      double car_y = j[1]["y"];
      double car_s = j[1]["s"];
      double car_d = j[1]["d"];
      double car_yaw = j[1]["yaw"];
      double speed_car = j[1]["speed"];

      // Previous path data given to the Planner
      vector<double> previous_path_x = j[1]["previous_path_x"];
      vector<double> previous_path_y = j[1]["previous_path_y"];
      // Previous path's end s and d values
      double end_path_s = j[1]["end_path_s"];
      double end_path_d = j[1]["end_path_d"];


//      std::cout << "prev= " << end_path_d << ", curr= " << car_d << std::endl;
//      std::cout << "speed_car= " << speed_car << std::endl;

      // Data of all enemies
      vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

      size_t prev_size = previous_path_x.size();

      auto car_s_cur = car_s;
      if (prev_size > 0) {
        car_s = end_path_s;
      }

      // Initialize lanes and lane handler
      LaneHandler laneHandler;
      for (int i = 0; i < 3; ++i) {
        Lane a_lane(i);
        laneHandler.lanes.push_back(a_lane);
      }

      int lane_car = frenetDToLane(car_d);
      double speed_delta = 0;

      if (lane_car != -1) {
        // Check the availabilty of each lane using sensor fusion data
        // and calculate costs
        for (auto &data_enemy : sensor_fusion) {
          double car_s_enemy = data_enemy[5];
          auto car_s_dif_with_enemy = car_s_enemy - car_s_cur;
          // Disregard far and irrelevant enemies
          if (car_s_dif_with_enemy < -50 || car_s_dif_with_enemy > 1000)
            continue;

          double vx = data_enemy[3];
          double vy = data_enemy[4];
          double speed_enemy = sqrt(vx * vx + vy * vy);
          car_s_enemy += ((double) prev_size * 0.02 * speed_enemy);

          double car_d_enemy = data_enemy[6];
          int lane_enemy = frenetDToLane(car_d_enemy);
          if (lane_enemy < 0 || lane_enemy > 3)
            continue;

          double s_distance_to_enemy = car_s_enemy - car_s;
//        std::cout << "s_distance_to_enemy = " << s_distance_to_enemy
//                  << std::endl;

          Lane &lane_enemy_is_on = laneHandler.getLane(lane_enemy);
          if (s_distance_to_enemy >= 0) {
            // Enemy is in front of us
            if (s_distance_to_enemy <= 20) {
              // Enemy is too close
              lane_enemy_is_on.isAvailable = false;
            } else if (s_distance_to_enemy <= 45 && speed_enemy < speed_car) {
              // Enemy has reasonable distance but is slow
              lane_enemy_is_on.isAvailable = false;
            }
            if (lane_enemy_is_on.nearestForwardEnemyDist >
                s_distance_to_enemy) {
              lane_enemy_is_on.nearestForwardEnemyDist = s_distance_to_enemy;
              lane_enemy_is_on.nearestForwardEnemyVelocity = speed_enemy;
            }
          } else if (lane_car != lane_enemy &&
                     (speed_enemy > speed_car || s_distance_to_enemy >= -30)) {
            // Enemy is in behind
            // Enemy is not in our lane
            // Enemy is close or faster than us
            lane_enemy_is_on.isAvailable = false;
          }
        }


        std::cout << std::fixed;
        std::cout << std::setprecision(3);
        for (int i = 0; i < laneHandler.lanes.size(); ++i) {
          std::cout << laneHandler.lanes[i].isAvailable << " ";
        }
        std::cout << " ||| ";
        for (int i = 0; i < laneHandler.lanes.size(); ++i) {
          std::cout << laneHandler.lanes[i].Cost() << " ";
        }
        std::cout << std::endl;


        bool lane_change_is_allowed = true;
        double time_delta =
          watch.ElapsedMilliSeconds() - lastTimeLaneChangeHappened;
        if (time_delta < 4000) {
          // If lane change happened, cooldown for x ms
          lane_change_is_allowed = false;
          //std::cout << "lane change isn't allowed " << time_delta << std::endl;
        }


        // Now that we have calculated costs and availability of each
        // lane, we can choose the right lane to drive
        int lane_min_cost = laneHandler.getLowestCostLane(lane_car);
        bool decelerate = false;

        if (lane_min_cost == lane_car) {
          if (!laneHandler.getLane(lane_car).isAvailable) {
            decelerate = true;
          }
        } else if (lane_change_is_allowed) {
          // Change Lane
          lane = lane_min_cost;
          std::cerr << "lane changed!!!!!!!!!" << std::endl;
          lastTimeLaneChangeHappened = watch.ElapsedMilliSeconds();
        }

        // Speed control
        if (ref_vel <= 32) {
          speed_delta = 10;
        } else if (decelerate) {
          double s_dist_front = laneHandler.getLane(
            lane_car).nearestForwardEnemyDist;
          if (laneHandler.getLane(lane_car).nearestForwardEnemyVelocity <
              speed_car) {
            // Enemy in front of us is too slow
            if (s_dist_front < 25) {
              // Wow enemy is soooo close
              speed_delta = -5;
            } else if (s_dist_front < 45) {
              speed_delta = -3;
            } else if (s_dist_front < 55) {
              speed_delta = -1;
            }
          }
        } else if (ref_vel < 49.5) {
          speed_delta = 2;
        }
      }

      //Create a list of widely spaced waypoints, evenly spaced at 30m
      vector<double> ptsx;
      vector<double> ptsy;

      double ref_x = car_x;
      double ref_y = car_y;
      double ref_yaw = deg2rad(car_yaw);

      if (prev_size < 2) {
        //Initialize
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
      } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
      }


      auto currentD = end_path_d;
      auto targetD = 2 + 4 * lane;
      auto delta = (targetD - currentD);

      // In Frenet add 30m spaced points in front of the starting reference
      vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane),
                                      map_waypoints_s, map_waypoints_x,
                                      map_waypoints_y);
      vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane),
                                      map_waypoints_s, map_waypoints_x,
                                      map_waypoints_y);
      vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane),
                                      map_waypoints_s, map_waypoints_x,
                                      map_waypoints_y);

      ptsx.push_back(next_wp0[0]);
      ptsx.push_back(next_wp1[0]);
      ptsx.push_back(next_wp2[0]);

      ptsy.push_back(next_wp0[1]);
      ptsy.push_back(next_wp1[1]);
      ptsy.push_back(next_wp2[1]);

      for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) -
                   shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) +
                   shift_y * cos(0 - ref_yaw));
      }


      // These will be fed to the simulator
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      // Use path from past
      for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
      }

      // Generate the spline
      tk::spline spline;
      spline.set_points(ptsx, ptsy);
      double target_x = 30;
      double target_y = spline(target_x);
      double target_dist = sqrt(
        (target_x) * (target_x) + (target_y) * (target_y));
      double x_add_on = 0;

      //Generate future points with a spline
      double speedChange = speed_delta / 72;
      for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        ref_vel += speedChange;
        double N = (target_dist / (.02 * ref_vel / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Switch to local coordinate frame
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
      }

      json json_msg;
      json_msg["next_x"] = next_x_vals;
      json_msg["next_y"] = next_y_vals;

      auto msg = "42[\"control\"," + json_msg.dump() + "]";

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}