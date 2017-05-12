#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <limits>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steer, pid_speed;
  // TODO: Initialize the pid variable.
  pid_steer.Init(0.2, 0.006, 3.);
  pid_speed.Init(0.006, 0.00001, 0.0001);

  bool use_twiddle = false;
  std::deque<double> angle_history;
  double twiddle_tol = 0.0002;
  double twiddle_best = std::numeric_limits<double>::max();
  double twiddle_err = 0;
  double twiddle_p[] = { 0.0002, 0.00001, 0.0001 };
  int twiddle_steps = 1000;
  int twiddle_num = 0;
  int twiddle_try = 0;
  int twiddle_idx = 0;

  h.onMessage([&pid_steer, &pid_speed, &angle_history, &use_twiddle,
  &twiddle_tol, &twiddle_steps, &twiddle_num, &twiddle_best,
  &twiddle_try, &twiddle_err, &twiddle_p, &twiddle_idx
  ](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double speed_cte;
          double target_speed;
          double throttle;

          /**
           * twiddle - throttle
           */
          if (use_twiddle && twiddle_num == 0) {
            if (twiddle_idx == 0) { pid_speed.Kp += twiddle_p[0]; }
            if (twiddle_idx == 1) { pid_speed.Ki += twiddle_p[1]; }
            if (twiddle_idx == 2) { pid_speed.Kd += twiddle_p[2]; }
            std::cout << " = " << twiddle_idx << "  "
                      << " Kp: " << pid_speed.Kp
                      << " Ki: " << pid_speed.Ki
                      << " Kd: " << pid_speed.Kd
                      << std::endl;
          }

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // Steer
          pid_steer.UpdateError(cte);
          steer_value = pid_steer.TotalError();
          // use sigmoid to limit the value between 1 and -1
          steer_value = 2 / (1 + exp(-steer_value)) -1;

          // Speed
          angle_history.push_back(angle);
          if (angle_history.size() > 10)
              angle_history.pop_front();

          // smooth out the angle
          double avg_angle = angle;
          if (angle_history.size() > 0) {
              avg_angle = std::accumulate(
                  angle_history.begin(),
                  angle_history.end(), 0.0) /
                  angle_history.size();
          }

          // Target speed: faster when angle is small
          target_speed = 90 - fabs(avg_angle) * 10;
          target_speed = fmax(target_speed, 15);

          speed_cte = speed - target_speed;
          pid_speed.UpdateError(speed_cte);
          throttle = 0.5 + pid_speed.TotalError();

          if (twiddle_num % 10 == 0) {
            //std::cout << " Angle: " << avg_angle
            //          << " Taget: " << target_speed
            //          << " Speed: " << speed
            //          << " Throttle: " << throttle
            //          << std::endl;
          }

          /**
           * Twiddle - throttle
           * It is not easy to use twiddle here for several reasons
           * 1. When the speed is too fast, the car run out of the
           *    track. We need a better way to detect that and reset
           *    the simulator.
           * 2. The steering angle should react differently when
           *    speed is different, so using two PID controller
           *    separately is really difficult to tweak the params.
           */
          if (use_twiddle) {
            twiddle_err += speed_cte * speed_cte;
            if ((twiddle_num % 100) == 0) std::cout << twiddle_err / twiddle_num << std::endl;
            if (twiddle_num == twiddle_steps) {
              twiddle_err /= twiddle_steps;
              if (twiddle_err < twiddle_best) {
                twiddle_best = twiddle_err;
                twiddle_p[twiddle_idx] *= 1.1;
                twiddle_idx += 1;
                twiddle_idx%= 3;
              }
              else {
                if (twiddle_try == 0) {
                  if (twiddle_idx == 0) { pid_speed.Kp -= 3*twiddle_p[0]; }
                  if (twiddle_idx == 1) { pid_speed.Ki -= 3*twiddle_p[1]; }
                  if (twiddle_idx == 2) { pid_speed.Kd -= 3*twiddle_p[2]; }
                  twiddle_try = 1;
                }
                else {
                  if (twiddle_idx == 0) { pid_speed.Kp += twiddle_p[0]; twiddle_p[0] *= 0.9; }
                  if (twiddle_idx == 1) { pid_speed.Ki += twiddle_p[1]; twiddle_p[1] *= 0.9; }
                  if (twiddle_idx == 2) { pid_speed.Kd += twiddle_p[2]; twiddle_p[2] *= 0.9; }
                  twiddle_try = 0;
                  twiddle_idx += 1;
                  twiddle_idx %= 3;
                }
              }

              if (twiddle_idx == 0) {
                // check tolerance
                double sum = twiddle_p[0] + twiddle_p[1] + twiddle_p[2];
                std::cout << "Delta: " << twiddle_p[0]
                          << " , " << twiddle_p[1]
                          << " , " << twiddle_p[2] << std::endl;
                std::cout << "Solution: "
                          << " Kp: " << pid_speed.Kp
                          << " Ki: " << pid_speed.Ki
                          << " Kd: " << pid_speed.Kd
                          << std::endl;
                // reset
                std::string msg = "42[\"reset\", {}]";
                //std::cout << msg << std::endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
              twiddle_num = 0;
            }
            else {
              twiddle_num += 1;
            }
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Angle: " << angle << " Speed: " << speed << " Target: " << target_speed << std::endl;
          std::cout << "Throttle: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
