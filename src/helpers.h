#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "spline.h"
#include <cmath>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double>& maps_x,
    const vector<double>& maps_y) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); ++i) {
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

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double>& maps_x,
    const vector<double>& maps_y) {
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 2) {
        ++closestWaypoint;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
    const vector<double>& maps_x,
    const vector<double>& maps_y) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
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
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return { frenet_s,frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double>& maps_s,
    const vector<double>& maps_x,
    const vector<double>& maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
        ++prev_wp;
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

    return { x,y };
}


double get_lane_cost(int lane, double car_speed, int predicted_lane, double predicted_ref_vel) {
    
    //   cost of speed
    double target_speed = 49.5 / 2.24;
    double cost_speed = 0;
    if (predicted_ref_vel < target_speed)
        cost_speed = 0.8 * (target_speed - predicted_ref_vel) / target_speed;
    else {
        if (car_speed > 50)
            cost_speed = 1;
        else
            cost_speed = (predicted_ref_vel - target_speed) / 0.5;
    }

    // cost of acceleration  
    double acceleration = (predicted_ref_vel - car_speed) / 0.02;
    double cost_acc = 0;
    if (acceleration < -9.8 || acceleration >9.8)
        cost_acc = 1;
    else
        cost_acc = abs(acceleration) / 9.8;

    // cost of lane change
    double cost_lane_change = abs(lane - predicted_lane) / 3;


    return 10 * cost_lane_change + 10 * cost_acc + 5 * cost_speed;
}

vector<vector<double>> get_trajectory(int lane, double ref_vel, double car_x, double car_y, double car_yaw, double car_s, double car_d,
    const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y,
    vector<double> previous_path_x, vector<double> previous_path_y) {

    //int prev_size = previous_path_x.size();
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
    vector<double> ptsx;
    vector<double> ptsy;
    // find two points in the past
    if (previous_path_x.size() < 2) {    //Use two points thats makes path tangent to the car
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else {        
        ref_x = previous_path_x[previous_path_x.size() - 1];
        ref_y = previous_path_y[previous_path_x.size() - 1];
        double ref_x_prev = previous_path_x[previous_path_x.size() - 2];
        double ref_y_prev = previous_path_y[previous_path_x.size() - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
    // get 3 more points
    for (int i = 1; i < 4; i++) {
        vector<double> next_wp = getXY(car_s + 30 * i, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }
    // move to car coordinates
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
        ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
    }
    // build the spline
    tk::spline s;
    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    //For the smooth transition, we are adding previous path points
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    for (int i = 1; i < 50 - previous_path_x.size(); i++) {

        double N = target_dist / (0.02 * ref_vel);
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //Rotate back to normal after rotating it earlier
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    vector<vector<double>> next_vals;
    next_vals.push_back(next_x_vals);
    next_vals.push_back(next_y_vals);

    return next_vals;
}
#endif  // HELPERS_H
