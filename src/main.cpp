#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <utility>
#include <algorithm> 

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        std::istringstream iss(line);
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

    int lane = 1;
    double ref_vel = 0;

    h.onMessage([&lane, &ref_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
        &map_waypoints_dx, &map_waypoints_dy]
        (uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
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
                            // j[1] is the data JSON object

                            // Main car's localization Data
                            double car_x = j[1]["x"];
                            double car_y = j[1]["y"];
                            double car_s = j[1]["s"];
                            double car_d = j[1]["d"];
                            double car_yaw = j[1]["yaw"];
                            double car_speed = j[1]["speed"];

                            // Previous path data given to the Planner
                            auto previous_path_x = j[1]["previous_path_x"];
                            auto previous_path_y = j[1]["previous_path_y"];
                            // Previous path's end s and d values 
                            double end_path_s = j[1]["end_path_s"];
                            double end_path_d = j[1]["end_path_d"];

                            // Sensor Fusion Data, a list of all other cars on the same side 
                            //   of the road.
                            auto sensor_fusion = j[1]["sensor_fusion"];
                            /**
                                * TODO: define a path made up of (x,y) points that the car will visit
                                *   sequentially every .02 seconds
                                */

                            if (previous_path_x.size() > 0)
                                car_s = end_path_s;

                            double front_car_speed;
                            bool is_car_on_left = false;
                            bool is_car_in_front = false;
                            bool is_car_on_right = false;

                            for (int i = 0; i < sensor_fusion.size(); i++) {
                                double check_car_s = sensor_fusion[i][5];
                                // excluding the cars which are not surrounding ego car
                                if (abs(check_car_s - car_s) < 100) {
                                    double check_car_d = sensor_fusion[i][6];
                                    int car_lane = (int)(check_car_d / 4);
                                    // check if the car in front is too close
                                    double car_vx = sensor_fusion[i][3];
                                    double car_vy = sensor_fusion[i][4];
                                    double car_speed = sqrt(pow(car_vx, 2) + pow(car_vy, 2));
                                    check_car_s = check_car_s + previous_path_x.size() * 0.02 * car_speed;
                                    if (car_lane == lane) {
                                        if ((check_car_s > car_s) && (check_car_s < car_s + 30)) {
                                            is_car_in_front = true;
                                            front_car_speed = car_speed; // m/s
                                        }
                                    }
                                    else {
                                        if ((car_lane == lane - 1) && (check_car_s > car_s && (check_car_s - car_s) < 40 || check_car_s < car_s && (car_s - check_car_s) < 30) || lane == 0)
                                            is_car_on_left = true;
                                        if ((car_lane == lane + 1) && (check_car_s > car_s && (check_car_s - car_s) < 40 || check_car_s < car_s && (car_s - check_car_s) < 30) || lane == 2)
                                            is_car_on_right = true;
                                    }
                                }
                            }   

                            std::cout << "present state: " << lane << "   " << ref_vel << std::endl;
                            std::cout << "predictions: " << is_car_on_left << is_car_on_right << is_car_in_front << std::endl;

                            vector<std::pair<int, double>> predicted_lanes_ref_vels;
                            if (is_car_in_front) {
                                predicted_lanes_ref_vels.push_back(std::make_pair(lane, ref_vel - 5 * 0.02));
                                if (!is_car_on_left)
                                    predicted_lanes_ref_vels.push_back(std::make_pair(std::max(lane - 1, 0), std::min(49.5 / 2.24, ref_vel + 5 * 0.02)));
                                if (!is_car_on_right)
                                    predicted_lanes_ref_vels.push_back(std::make_pair(std::min(lane + 1, 2), std::min(49.5 / 2.24, ref_vel + 5 * 0.02)));
                            }
                            else {
                                predicted_lanes_ref_vels.push_back(std::make_pair(lane, std::min(49.5 / 2.24, ref_vel + 8 * 0.02)));
                            }

                            // Choose the best (i.e. lowest-cost) trajectory
                            double lowest_cost = 999999;
                            for (int i = 0; i < predicted_lanes_ref_vels.size(); i++) {
                                int predicted_lane = predicted_lanes_ref_vels[i].first;
                                double predicted_ref_vel = predicted_lanes_ref_vels[i].second;
                                // calculate cost
                                double lane_cost = get_lane_cost(lane, car_speed, predicted_lane, predicted_ref_vel);
                                // get the lowest cost and corresponding trajectory
                                if (lane_cost < lowest_cost) {
                                    lowest_cost = lane_cost;
                                    lane = predicted_lane;
                                    ref_vel = predicted_ref_vel;
                                }
                            }

                            std::cout << "next prediction:  " << lane << "     " << ref_vel << std::endl;
                            // Trajectory generation
                            vector<vector<double>> optimal_trajectory = get_trajectory(lane, ref_vel, car_x, car_y, car_yaw, car_s, car_d,
                                map_waypoints_s, map_waypoints_x, map_waypoints_y,
                                previous_path_x, previous_path_y);

                            json msgJson;
                            msgJson["next_x"] = optimal_trajectory[0];
                            msgJson["next_y"] = optimal_trajectory[1];

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }  // end "telemetry" if
                    }
                    else {
                        // Manual driving
                        std::string msg = "42[\"manual\",{}]";
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }  // end websocket if
        }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
        });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
        char* message, size_t length) {
            ws.close();
            std::cout << "Disconnected" << std::endl;
        });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    }
    else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
