#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) 
  {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from0
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    double Lf = 2.67;
    cout << "check 1";
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          cout << "here 1";
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          */

          // Shifting the car reference angle top 90 degree.

          auto ptsx_transform = Eigen::VectorXd(ptsx.size());
          auto ptsy_transform = Eigen::VectorXd(ptsx.size());
          for (int i=0; i < ptsx.size(); i++)
          {
            double shift_ptsx = ptsx[i] - px;
            double shift_ptxy = ptsy[i] - py;
            ptsx_transform[i] = (shift_ptsx * cos(0 - psi) - shift_ptxy * sin(0 - psi));
            ptsy_transform[i] = (shift_ptsx * sin(0 - psi) + shift_ptxy * cos(0 - psi));
          }
          cout << "here 2";
          auto coeffs = polyfit (ptsx_transform, ptsy_transform, 3);
          cout << "here 3";
           //calculate the cross track error.
          double cte = polyeval (coeffs , 0);
          cout << "here 4";
          //const double delay = 0.1;
          //calculate the orientation error
          double epsi = - atan(coeffs[1]);        
          cout << "here 5";  
          /* Both are in between [-1, 1].
          *
          */
          
          cout << "here 6";
          // Latency
          const double latency_evl = 0.1;

          const double px_now = 0.0 + (v * (cos(0.0)) * latency_evl);
          const double py_now = 0.0 + (v * (sin(0.0)) * latency_evl);
          const double psi_now = 0.0 + v * (-delta) / Lf * latency_evl;
          const double v_now = v + a * latency_evl;
          const double cte_now = coeffs[0] + v * sin(-atan(coeffs[1])) * latency_evl;
          const double epsi_now = -atan(coeffs[1]) - (v * atan(coeffs[1]) * latency_evl / Lf) ; 

          cout << "here 7";
          Eigen::VectorXd state(6);

          state << px_now, py_now, psi_now, v_now, cte_now, epsi;
          cout << "here 8";
          auto vars = mpc.Solve(state, coeffs);
          cout << "here vars";
          double steer_value = vars[0]/deg2rad(25);
          double throttle_value = vars[1];
          cout << "here 9";
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          //msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          cout << "here 10";
          for (int i=2; i<vars.size(); i++)
          {
            if ( i % 2 == 0 ) {
              mpc_x_vals.push_back( vars[i] );
            } 
            else 
            {
              mpc_y_vals.push_back( vars[i] );
            }
          }
          

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          double iteration = 25;

          for (unsigned int i=0; i < iteration; i++)
          {
            double x = poly_inc * i;
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs , x));
          }


          json msgJson;
          
          msgJson["steering_value"] = steer_value;
          msgJson["throttle value"] = throttle_value;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals; 


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
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
