/*                                                                         80->|
* main.cpp
*
* Modifications: James William Dunn (github.com/jwdunn1)
*          Date: Sepember 7, 2017
*          Bosch Challenge Version
*          Post submission update for final track

MIT License

Copyright(c) 2017 James William Dunn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/


#ifndef _WINSOCK13_
  //#define _WINSOCK13_
#endif

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "timer.h"
#include "spline.h"
#include <string>

using namespace std;
#define BOSCH           true
#define VELOCITYLOW     17.8816   // 40.0 mph
#define VELOCITYHIGH    22.173184 // 49.6 mph
#define MAXLANEDEPTH    99999.0
#define PASSDIST        3.0
#define STEERINGMAX     0.005
#define STEERINGMIN     0.005
#define MAX_S           6945.554  // the max s value before wrap to 0
#define RADIUSDLC       400.0     // threshold for double lane change
#define MAXD            0.44704
#define NORMALACCEL     0.003

enum STATELIST { Launch, Advance } STATE_ = Launch;
double dist_inc_max = 0.4462; // <-- updated by the behavior planner

// for convenience
using json = nlohmann::json;

// degrees to radians
double deg2rad(double x) { return x * 0.017453292519943295769; }
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}
double diff(double ref, double targ) {
  double hbase = MAX_S / 2.0;
  double t = targ - ref;
  return t>hbase ? t - MAX_S : (t<-hbase ? t + MAX_S : t);
}
// find radius given three side lengths of a triangle
double triRadius(double a, double b, double c) {
  return (a*b*c) / sqrt((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c));
}

// Double Lane Change on fairly straight segments
bool straightSegment(double car_s, double len, 
    tk::spline laneSplineX, tk::spline laneSplineY) {
  bool dlcEnabled = false;
  // gather five points on the center lane spline ahead
  double x1 = laneSplineX(car_s);
  double y1 = laneSplineY(car_s);
  double x2 = laneSplineX(car_s + len*0.25);
  double y2 = laneSplineY(car_s + len*0.25);
  double x3 = laneSplineX(car_s + len*0.5);
  double y3 = laneSplineY(car_s + len*0.5);
  double x4 = laneSplineX(car_s + len*0.75);
  double y4 = laneSplineY(car_s + len*0.75);
  double x5 = laneSplineX(car_s + len);
  double y5 = laneSplineY(car_s + len);
  // compute side distances and radius for each triangle
  double a = distance(x1, y1, x3, y3);
  double b = distance(x3, y3, x5, y5);
  double c = distance(x1, y1, x5, y5);
  double r1 = triRadius(a, b, c);
  a = distance(x1, y1, x2, y2);
  b = distance(x2, y2, x3, y3);
  c = distance(x1, y1, x3, y3);
  double r2 = triRadius(a, b, c);
  a = distance(x2, y2, x3, y3);
  b = distance(x3, y3, x4, y4);
  c = distance(x2, y2, x4, y4);
  double r3 = triRadius(a, b, c);
  a = distance(x3, y3, x4, y4);
  b = distance(x4, y4, x5, y5);
  c = distance(x3, y3, x5, y5);
  double r4 = triRadius(a, b, c);
  double r = min(min(r1,r2), min(r3,r4));
  if (r > RADIUSDLC) dlcEnabled = true;
  return dlcEnabled;
}

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

double dist_inc = 0.009; // distance increment per 20ms 
Timer1 timerFrame;
Timer1 timerLastLaneChange;
Timer1 timerDLC;
int frameCount = 0;
double LaneTarget[3] = { 2.0, 6.0, 10.0 }; // default lane
int curLane = 1;
int destLane = 1;
double begin_s = 85.3873; // <-- may be updated on first frame
double maxSenseDistance = 0;
double maxSenseVelocity = 0;

// Allocate a spline array
tk::spline laneSpline[3][2] = { {/*lane 0*/},{/*lane 1*/ },{/*lane 2*/ } };

// Persist state {x,y,s,a,inc} path data between messages
vector<array<double, 5>> prev_path;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read
  string map_file_ = "../data/highway_map_bosch1.csv";
  if (!BOSCH) map_file_ = "../data/highway_map.csv";
  cout << "Map file: " << map_file_ << endl;
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
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
  cout << "Waypoints read: " << map_waypoints_x.size() << endl;
  int wpsize = map_waypoints_x.size();

  // instantiate the lane spline array
  for (int i=0; i<3; i++) {
    vector<double> splinepoints_x; // a set of coordinates down the lane
    vector<double> splinepoints_y;
    vector<double> indexpoints;
    // begin the spline with the last master waypoint...
    splinepoints_x.push_back(map_waypoints_x[wpsize-1] 
      + map_waypoints_dx[wpsize - 1] * LaneTarget[i]);
    splinepoints_y.push_back(map_waypoints_y[wpsize-1] 
      + map_waypoints_dy[wpsize - 1] * LaneTarget[i]);
    indexpoints.push_back(-31.405);
    for (int j = 0; j < wpsize; j++) { // offset from master waypoints
      splinepoints_x.push_back(map_waypoints_x[j] 
        + map_waypoints_dx[j] * LaneTarget[i]);
      splinepoints_y.push_back(map_waypoints_y[j] 
        + map_waypoints_dy[j] * LaneTarget[i]);
      indexpoints.push_back(map_waypoints_s[j]);
      }
    // and end with the first point for a perfectly stitched loop
    splinepoints_x.push_back(map_waypoints_x[0] 
      + map_waypoints_dx[0] * LaneTarget[i]);
    splinepoints_y.push_back(map_waypoints_y[0] 
      + map_waypoints_dy[0] * LaneTarget[i]);
    indexpoints.push_back(MAX_S);

    laneSpline[i][0].set_points(indexpoints, splinepoints_x); // use S as index
    laneSpline[i][1].set_points(indexpoints, splinepoints_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
    &map_waypoints_dx,&map_waypoints_dy](
  #ifdef _WINSOCK13_
    uWS::WebSocket<uWS::SERVER> *ws, 
  #else
    uWS::WebSocket<uWS::SERVER> ws, 
  #endif
  char *data, size_t length,
                     uWS::OpCode opCode) {
  // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
    if (event == "telemetry") {
      // Assume low latency and start a stopwatch 
      if (frameCount == 1) timerFrame.restart();
      //cout << frameCount << endl;

      // j[1] is the data JSON object
      // Main car's localization Data
      double car_x = j[1]["x"];
      double car_y = j[1]["y"];
      double car_s = j[1]["s"];
      double car_d = j[1]["d"];

      ///////////////////////////////////////////////////////////////////
      // ACCOUNTING
      //
      double timeSince = (int(timerFrame.sinceBeginning() / 10.0)) / 100.0;

      assert(car_d >0.0 && car_d < 12.0); // must be on the track
      curLane = car_d < 4.0 ? 0 : (car_d < 8.0 ? 1 : 2); // 0, 1, or 2

      double car_yaw = j[1]["yaw"];
      double car_speed = j[1]["speed"];
      if (car_speed >= 50.0) cout << "Velocity violation. s: " << car_s;
      assert(car_speed<50.0); // must be less than 50mph
      Timer1::ms frameDuration = timerFrame.duration(); // frame-to-frame
      long outfd = frameDuration.count();
      // Previous path data given to the Planner
      auto previous_path_x = j[1]["previous_path_x"];
      auto previous_path_y = j[1]["previous_path_y"];
      // path size determined from simulator
      int path_size = previous_path_x.size(); 
      if (path_size == 0) {
        frameCount = 0;
        destLane = 1;   // or 0
        STATE_ = Launch;
        dist_inc = 0.009;
      }
      if (frameCount++ == 0) {
        cout << "________________________________________________" << endl;
        cout << car_s << ": Launching..." << endl;
        begin_s = car_s;
      }

      if (car_s - begin_s >= 2816.352 && car_s - begin_s <= 2817.352)
        cout << "car_s: " << car_s << " 1.75mi in t: " << timeSince << endl;

      // Previous path's end s and d values 
      double end_path_s = j[1]["end_path_s"];
      double end_path_d = j[1]["end_path_d"];

      // Sensor Fusion Data, a list of other cars on the same side of the road.
      auto sensor_fusion = j[1]["sensor_fusion"];


      ///////////////////////////////////////////////////////////////////
      // SENSOR DATA PROCESSING
      //

      // get closest obstacle id's and velocities
      int num_obstacle = sensor_fusion.size();
      bool replan = false;

      // 2 x 3 array of obstacles 
      //  Ahead: L,C,R   [[ 0, 0, 0 ],
      // Behind: L,C,R    [ 0 ,0, 0 ]]
      double min_dist[2][3] = {
        { MAXLANEDEPTH,MAXLANEDEPTH,MAXLANEDEPTH },
        { MAXLANEDEPTH,MAXLANEDEPTH,MAXLANEDEPTH }
      };
      int cars_ahead[3] = {0,0,0};
      double dist_inc_maxmps = dist_inc_max*50.0;
      double vel_obj[2][3] = {
        { dist_inc_maxmps, dist_inc_maxmps, dist_inc_maxmps },
        { dist_inc_maxmps, dist_inc_maxmps, dist_inc_maxmps }
      };

      for (int i = 0; i < num_obstacle; ++i) {
        auto sens = sensor_fusion[i];
        double sens_x = sens[1];
        double sens_y = sens[2];
        double sens_vel_x = sens[3]; // mps
        double sens_vel_y = sens[4];
        double sens_s = sens[5];
        double sens_d = sens[6];
        double sens_vel = sqrt(sens_vel_x*sens_vel_x + sens_vel_y*sens_vel_y);
        double dist = diff(car_s, sens_s);
        
        double sens_dist = abs(dist);
        int sens_lane = sens_d < 4.0 ? 0 : (sens_d < 8.0 ? 1 : 2); // 0, 1, or 2
        // 0 or 1 (0=ahead, 1=behind)  if dist neg, then it is behind
        int sens_pos = dist<0.0;
        // count of cars ahead
        if (sens_pos == 0 && sens_dist < 50.0) cars_ahead[sens_lane]++; 
        if (sens_dist > maxSenseDistance) maxSenseDistance = sens_dist;
        if (sens_vel > maxSenseVelocity) maxSenseVelocity = sens_vel;

        if (sens_dist < min_dist[sens_pos][sens_lane]) {
          vel_obj[sens_pos][sens_lane] = sens_vel;
          min_dist[sens_pos][sens_lane] = sens_dist;
        }

        // Check adjacent vehicles for intersection
        if (path_size > 9 &&
          ((sens_dist < 20.0 && sens_pos == 0) ||
          (sens_dist < 10.0 && sens_pos == 1))
          && abs(curLane - sens_lane) == 1) {
          for (int i=1; i<6; i++) { // step ahead from 0.2 to 1.0 sec
            double sens_fx = sens_x + sens_vel_x*0.2*i; // future state
            double sens_fy = sens_y + sens_vel_y*0.2*i;
            if (i*10 < path_size && // future state of reference vehicle
              distance(previous_path_x[10*i-1], previous_path_y[10*i-1], 
                sens_fx, sens_fy) < 1.0) { // paths cross 
              sens_lane = curLane;
              replan = true;
              cout << "Danger detected "
              << (sens_pos == 0 ? "ahead" : "behind")
              << ": " << sens_dist
              << "m, delta d: " << abs(car_d - sens_d) << "m" << endl;
            }
          }
        }
      }

      // find the maximum distance of the three lanes
      int prefLane = 1;
      double largerDist = 0;
      for (int i = 0; i < 3; i++) {
        if (min_dist[0][i] > largerDist) {
          prefLane = i;
          largerDist = min_dist[0][i];
        }
      }


      /////////////////////////////////////////////////////////////////////////
      // BEHAVIOR PLANNER
      //

      double dist_inc_acc = NORMALACCEL;
      double dist_inc_mps = dist_inc*50.0;
      double steer_limit = STEERINGMIN;
      dist_inc_max = VELOCITYHIGH / 50.0;

      double dist_inc_Goal = dist_inc_max;

      // Determine if over a lane line
      bool on_lane_line = (car_d>3.0 && car_d<5.0) || (car_d>7.0 && car_d<9.0);
      bool dlc = false;

      // Manage state with an FSM
      switch (STATE_) {
        case Launch:
          if (dist_inc*50.0 >= VELOCITYHIGH-0.001) {
              cout << "t: " << timeSince << " s: " << car_s 
                << ": Switching to Go mode" << endl;
              STATE_ = Advance;
          }
          break;
        case Advance: // Go mode
          break;
      }

      // Short-range planning
      bool braking = false;
      if (min_dist[0][curLane] < (cars_ahead[curLane]==1?20.0:36.0) 
        && curLane == destLane) { // override for LC
        double brakePercent = // Braking function
          (86.53962 / 0.2473242) * (1 - exp(-0.2473242 
            * (min_dist[0][curLane]-4.0 ) )) - 249.4352;
        brakePercent = max(0.0, min(100.0, brakePercent));
        brakePercent = brakePercent / 100.0;
        if (brakePercent < 0.8) {
          replan = true;
          dist_inc_acc = 0.003368; // maximum deceleration
        }
        // reduce to % of obstacle vel over 50 frames
        dist_inc_Goal = brakePercent * vel_obj[0][curLane] / 50.0; 
        if (brakePercent < 0.999) braking = true;
        if (dist_inc_Goal > dist_inc_max) {
          dist_inc_Goal = dist_inc_max;
          cout << "t: " << timeSince << " s: " << car_s
            << " Obstacle velocity: " << vel_obj[0][curLane] << endl ;
        }
      }
      
      // Medium-range planning
      if (!replan && min_dist[0][curLane] < 35.0 
        && timerLastLaneChange.since().count() > 2000
        && timerDLC.since().count() > 5000) {

        // Lane 1 -> 0 or 2 (whichever is preferred)
        if (curLane != prefLane && prefLane != 1 && curLane == 1
          && min_dist[0][prefLane] > min_dist[0][curLane]+3.0
          && cars_ahead[prefLane] <= 1
          && min_dist[1][prefLane] > max(PASSDIST, 
            1.0*(vel_obj[1][prefLane] - dist_inc_mps) + PASSDIST)) {
          destLane = prefLane;
          cout << "t: " << timeSince << " s: " << car_s 
            << " Changing to deeper lane " << prefLane << endl;
          timerLastLaneChange.start();
          replan = true;
          if (!braking) dist_inc_Goal = dist_inc_max; // and speed up
        } else
      
        // Lane 0 -> 1
        if (curLane == 0 && min_dist[0][1] > min_dist[0][curLane] + 3.0
            && min_dist[1][1] > max(PASSDIST, 
              1.0*(vel_obj[1][1] - dist_inc*50.0)+PASSDIST)) {
          destLane = 1;
          cout << "t: " << timeSince << " s: " << car_s 
            << ": Changing to lane 1" << endl;
          timerLastLaneChange.start();
          replan = true;
          if (!braking) dist_inc_Goal = dist_inc_max;
        } else
        
        // Lane 2 -> 1
        if (curLane == 2 && min_dist[0][1] > min_dist[0][curLane] + 3.0
            && min_dist[1][1] > max(PASSDIST, 
              1.0*(vel_obj[1][1] - dist_inc*50.0)+PASSDIST)) {
          destLane = 1;
          cout << "t: " << timeSince << " s: " << car_s 
            << ": Changing to lane 1" << endl;
          timerLastLaneChange.start();
          replan = true;
          if (!braking) dist_inc_Goal = dist_inc_max;
        } else

        // Lane 0 -> 2 (also check 1) DOUBLE LC
        if (curLane == 0 && min_dist[0][1] > 10.0 && cars_ahead[1] == 1
          && min_dist[1][1] > max(11.0, 
            1.0*(vel_obj[1][1] - dist_inc_mps) + 11.0)
          && min_dist[0][2] > 70.0 && max(11.0, 
            2.0*(vel_obj[1][2] - dist_inc_mps) + 11.0)) {
          if (straightSegment(car_s, 120.0, laneSpline[1][0], laneSpline[1][1])) {
            destLane = 2;
            cout << "t: " << timeSince << " s: " << car_s 
              << " Going around to lane 2" << endl;
            timerDLC.start();
            replan = true;
            steer_limit = 0.00245; // easy on the steering
            if (!braking) dist_inc_Goal = dist_inc_max;
          }
        } else

        // Lane 2 -> 0 (also check 1) DOUBLE LC
        if (curLane == 2 && min_dist[0][1] > 10.0 && cars_ahead[1] <= 1
          && min_dist[1][1] > max(11.0, 
            1.0*(vel_obj[1][1] - dist_inc_mps) + 11.0)
          && min_dist[0][0] > 70.0 && max(11.0, 
            2.0*(vel_obj[1][0] - dist_inc_mps) + 11.0)) {
          if (straightSegment(car_s, 120.0, laneSpline[1][0], laneSpline[1][1])) {
            destLane = 0;
            cout << "t: " << timeSince << " s: " << car_s 
              << " Going around to lane 0" << endl;
            timerDLC.start();
            replan = true;
            steer_limit = 0.00245; // easy on the steering
            if (!braking) dist_inc_Goal = dist_inc_max;
          }
        }
      }
      
      // Long-range planning 
      if (!replan && min_dist[0][curLane] > 35.0 && min_dist[0][curLane] < 100.0
        && timerLastLaneChange.since().count() > 2000
        && timerDLC.since().count() > 5000) {
          
        // Lane 0 or 2 -> 1 preference to stay in middle
        if (((curLane == 2 && min_dist[0][1] > 200.0 && min_dist[1][1] > 21.0)
            || (curLane == 0 && min_dist[0][1] > 200.0 && min_dist[1][1] > 21.0)) ) {
          destLane = 1;
          cout << "t: " << timeSince << " s: " << car_s 
            << ": Returning to lane 1" << endl;
          timerLastLaneChange.start();
        } else
          
        // Lane 1 -> 0 early lane change around upcoming traffic
        if ( ((curLane == 1 && min_dist[0][0] > 200.0 && min_dist[1][0] > 21.0) 
          && min_dist[0][1] < 50.0) ) {
          destLane = 0;
          cout << "t: " << timeSince << " s: " << car_s 
            << ": Early switch to lane 0" << endl;
          timerLastLaneChange.start();
        } else
          
        // Lane 1 -> 2 early lane change around upcoming traffic
        if (((curLane == 1 && min_dist[0][2] > 200.0 && min_dist[1][2] > 21.0)
            && min_dist[0][1] < 50.0) ) {
          destLane = 2;
          cout << "t: " << timeSince << " s: " << car_s 
            << ": Early switch to lane 2" << endl;
          timerLastLaneChange.start();
        } else

        // Lane 0 -> 2 (also check 1) LR DOUBLE LC
        if (curLane == 0 && min_dist[0][1] > 40.0
          && min_dist[1][1] > max(11.0, 3.0*(vel_obj[1][1] - dist_inc_mps) + 11.0)
          && min_dist[0][2] > 100.0 && min_dist[1][2] > 20.0) {
          if (straightSegment(car_s, 120.0, laneSpline[1][0], laneSpline[1][1])) {
            destLane = 2;
            cout << "t: " << timeSince << " s: " << car_s 
              << " Long around to lane 2" << endl;
            timerDLC.start();
            steer_limit = 0.00245; // feather the steering
            dist_inc_Goal = dist_inc_max;
            dlc = true;
          }
        }
        else

        // Lane 2 -> 0 (also check 1) LR DOUBLE LC
        if (curLane == 2 && min_dist[0][1] > 40.0
          && min_dist[1][1] > max(11.0, 3.0*(vel_obj[1][1] - dist_inc_mps) + 11.0)
          && min_dist[0][0] > 100.0 && min_dist[1][0] > 20.0) {
          if (straightSegment(car_s, 120.0, laneSpline[1][0], laneSpline[1][1])) {
            destLane = 0;
            cout << "t: " << timeSince << " s: " << car_s 
              << " Long around to lane 0" << endl;
            timerDLC.start();
            steer_limit = 0.00245; // feather the steering
            dist_inc_Goal = dist_inc_max;
            dlc = true;
          }
        }

      }


      ///////////////////////////////////////////////////////////////////
      // MOTION CONTROLLER
      //

      vector<double> next_x_vals;
      vector<double> next_y_vals;

      double pos_x;
      double pos_y;
      double pos_s;
      double angle = 0.0;

      if (path_size == 0) {
        pos_x = car_x;
        pos_y = car_y;
        pos_s = car_s;
        angle = deg2rad(car_yaw);
      }
      else {
        assert(path_size>1); // must be 2 or more items on the path

        // remove expired items from the previous path
        int prevlength = prev_path.size();
        prev_path.erase(prev_path.begin(), prev_path.begin()+(prevlength-path_size));
        assert(prev_path.size() == path_size);  // double check erasure

        // Old plan is not acceptable, replace with a new one
        if (replan) {
          path_size = 4;
        }

        // reuse previous path
        for (int i = 0; i < path_size; i++) { 
          next_x_vals.push_back(prev_path[i][0]);
          next_y_vals.push_back(prev_path[i][1]);
        }

        if (replan) {
          prev_path.erase(prev_path.begin()+path_size, prev_path.end());
          assert(prev_path.size() == path_size);
          dist_inc = prev_path[path_size - 1][4];
          assert(dist_inc < MAXD); // sanity check
          end_path_s = prev_path[path_size - 1][2];
        }
        pos_x = prev_path[path_size-1][0];
        pos_y = prev_path[path_size-1][1];
        pos_s = end_path_s;
        angle = prev_path[path_size-1][3];
      }

      int frameCnt = (dlc?117:50) - path_size;
      
      for (int i = 0; i < frameCnt; i++) {
        double refAngle = 0.0, steering = 0.0;

        pos_s += dist_inc;
        if (pos_s > MAX_S) pos_s -= MAX_S; // modulo

        if (begin_s - pos_s > 4350.0) // end report
          cout << "maxSenseVelocity: " << maxSenseVelocity 
          << " maxSenseDistance: " << maxSenseDistance << endl;

        // target a point several meters ahead in the current lane
        double refTarget = pos_s + 12.5;
        if (refTarget > MAX_S) refTarget -= MAX_S; // modulo

        double wpoffsetx = laneSpline[destLane][0](refTarget);
        double wpoffsety = laneSpline[destLane][1](refTarget);
        
        // transform lane waypoint to vehicle coordinates
        double dx = wpoffsetx - pos_x; // translate
        double dy = wpoffsety - pos_y;

        double cosPsi = cos(-angle);
        double sinPsi = sin(-angle);
        wpoffsetx = dx * cosPsi - dy * sinPsi; // rotate
        wpoffsety = dy * cosPsi + dx * sinPsi;

        // find bearing to waypoint
        refAngle = atan2(wpoffsety, wpoffsetx);
        if (refAngle == 0.0) cout << "dead straight!" << endl;
        steering = refAngle;

        // if at max velocity, go to larger steering value
        if (dist_inc == dist_inc_max && steer_limit == STEERINGMIN)
          steer_limit = STEERINGMAX;

        if (refAngle > 0.0) { // steer left
          if (steering > steer_limit)
            steering = steer_limit;
        }
        if (refAngle < 0.0) { // steer right
          if (steering < -steer_limit)
            steering = -steer_limit;
        }

        // Launch / stop conditions
        if (STATE_ == Launch) { // back off the throttle when steering
          dist_inc_acc = -0.052*steering + 0.003;  //  0.00365
        } else if (dist_inc < 0.05) {
          steering = 0.0; // too slow, don't steer
        }

        angle += steering;
        double last_x = pos_x;
        double last_y = pos_y;
        pos_x += (dist_inc)*cos(angle);
        pos_y += (dist_inc)*sin(angle);

        // accelerating
        if (dist_inc < dist_inc_Goal) {
          dist_inc += dist_inc_acc;
          if (dist_inc > dist_inc_Goal) dist_inc = dist_inc_Goal;
        }
        // decelerating
        if (dist_inc > dist_inc_Goal) {
          dist_inc -= dist_inc_acc;
          if (dist_inc < dist_inc_Goal) dist_inc = dist_inc_Goal;
        }

        // push trajectory data to the simulator
        next_x_vals.push_back(pos_x);
        next_y_vals.push_back(pos_y);
        // ...and save it for next cycle
        prev_path.push_back({ pos_x,pos_y,pos_s,angle,dist_inc });
      }
      
      json msgJson;
      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

      auto msg = "42[\"control\","+ msgJson.dump()+"]";

      //this_thread::sleep_for(chrono::milliseconds(1000));
#ifdef _WINSOCK13_
      ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
    }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef _WINSOCK13_
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }
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

  h.onConnection([&h](
#ifdef _WINSOCK13_
  uWS::WebSocket<uWS::SERVER> *ws, 
#else
  uWS::WebSocket<uWS::SERVER> ws,
#endif
  uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](
#ifdef _WINSOCK13_
    uWS::WebSocket<uWS::SERVER> *ws, 
#else
    uWS::WebSocket<uWS::SERVER> ws,
#endif
    int code, char *message, size_t length) {
#ifdef _WINSOCK13_
    ws->close();
#else
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
#ifdef _WINSOCK13_
  if (h.listen("0.0.0.0",port)) {
#else
  if (h.listen(port)) {
#endif
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
// eof
