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
using std::cout;
using std::endl;
using Eigen::VectorXd;

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
  if (found_null != string::npos) {
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
// Adapted from
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

class MPController(){
  public:
  MPController(){
    ros::init("control_node");
    ros::NodeHandle n;

    //variable initializations
    vector<double> ptsx;
    vector<double> ptsy;
    double px = state.pose[11].position.x;                                                                    //get the positions of the car variable
    double py = state.pose[11].position.y;                                                                    
    double psi = 0;
    double v = 0;

    // MPC is initialized here!
    MPC mpc;

    ros::Publisher acc_pub = n.advertise<std_msgs::Float64>("/throttle_cmd", 1000);
    ros::Publisher brake_pub = n.advertise<std_msgs::Float64>("/brake_cmd", 1000);
    ros::Publisher steering_pub = n.advertise<std_msgs::Float64>("/steering_cmd", 1000);
    ros::Publisher gear_pub = n.advertise<std_msgs::UInt8>("/gear_cmd", 1000);

    //subscribe to call topics.
    ros::Subscriber path_sub = n.subscribe("/path", 1000, path_callback);
    ros::Subscriber vel_sub = n.subscribe("/velocity_array", 1000, vel_callback);
    ros::Subscriber state_sub = n.subscribe("/gazebo/model_states", 1000, state_callback);
    ros::Subscriber steer_sub = n.subscribe("/current_steer_angle", 1000, steer_callback);
    ros::Subscriber control_sub = n.subscribe("/path", 1000, control_callback);

    std_msgs::Float64 throttle;
    std_msgs::Float64 brake;
    std_msgs::Float64 steering;
    std_msgs::UInt8 gear;

  }

  void path_callback(const nav_msgs::Path::ConstPtr& path){
    bool path_repeat = False;                                                         //implement path repeat feature from control_node(during testing)
    l = len(path.poses);

    int waypoints_number;
    VectorXd vehicle_waypoints_x(l);
    VectorXd vehicle_waypoints_y(l);

    if(!path_repeat){
      for(int j = 0; j < l; j++){
        taking in all the waypoints on topic 
        ptsx.push_back(path.poses[j].pose.position.x);
        ptsy.push_back(path.poses[j].pose.position.y);
        
        //helps set waypoints to the car.
        double diff_x = ptsx[j] - px;
        double diff_y = ptsy[j] - py;
        vehicle_waypoints_x[j] = diff_x * cos(-psi) - diff_y * sin(-psi);
        vehicle_waypoints_y[j] = diff_y * cos(-psi) + diff_x * sin(-psi);
      }
    }
  }

  void vel_callback(const nav_msgs::Path::ConstPtr& vel_arr){
      self.velocities.append(vel_arr.data[0]);

      l = len(vel_arr.data);
      //vel = np.zeros((l, 1))
      for(int j = 0; j < l; j++){ 
         vel[i][0] = vel_arr.data[i];
      }

      //self.lin_val_v(vel)
  }

  void state_callback(const nav_msgs::Path::ConstPtr& state){
        px = state.pose[11].position.x;
        py = state.pose[11].position.y;

        pose_arr.append(state.pose[-1].position.x);
        pose_arr.append(state.pose[-1].position.y);

        twist_arr.append(state.twist[-1].linear.x);
        twist_arr.append(state.twist[-1].linear.y);
        twist_arr.append(state.twist[-1].angular.z);

        /*        self.x_0 = vertcat(   [pose_arr[0]],
                              [pose_arr[1]],
                              [float(np.sqrt(twist_arr[0]**2 + twist_arr[1]**2))],
                              [float(np.arctan(twist_arr[1]/(twist_arr[0]+(1e-50))))],
                              [float(twist_arr[2])],
                              [self.steer],
                              [self.z_sim]
                            )
        */
 
  }

  void steer_callback(const nav_msgs::Path::ConstPtr& steer){
    //self.steer = steer.data
   }

  void control_callback(const nav_msgs::Path::ConstPtr& control){

  }
    
}


int main() {
  MPController controller;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
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

          cout << "ptsx[0]: " << ptsx[0] << " ptsy[0]: " << ptsy[0] << endl;
          cout << "ptsx[1]: " << ptsx[1] << " ptsy[1]: " << ptsy[1] << endl;
          cout << "ptsx[2]: " << ptsx[2] << " ptsy[2]: " << ptsy[2] << endl;
          cout << "ptsx[3]: " << ptsx[3] << " ptsy[3]: " << ptsy[3] << endl;
          cout << "ptsx[4]: " << ptsx[4] << " ptsy[4]: " << ptsy[4] << endl;
          cout << "ptsx[5]: " << ptsx[5] << " ptsy[5]: " << ptsy[5] << endl;
          cout << "" << endl;

          cout << "****Status****" << endl;
          cout << "px: " << px << endl;
          cout << "py: " << py << endl;
          cout << "psi: " << psi << endl;
          cout << "v: " << v << endl;

          // Affine transformation. Consider car's orientation
          int waypoints_number = ptsx.size();
          VectorXd vehicle_waypoints_x(waypoints_number);
          VectorXd vehicle_waypoints_y(waypoints_number);
          for (int i = 0; i < waypoints_number; i++) {
            double diff_x = ptsx[i] - px;
            double diff_y = ptsy[i] - py;
            vehicle_waypoints_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
            vehicle_waypoints_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
          }
          auto coeffs = polyfit(vehicle_waypoints_x, vehicle_waypoints_y, 3);
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          Eigen::VectorXd state(6);
          // Car's coordinate
          state << 0, 0, 0, v, cte, epsi;
          // Pass current state, reference trajectory's coefficients and get next actuator inputs
          // T is determined in MPC module
          auto vars = mpc.Solve(state, coeffs);
          steer_value = vars[0];
          throttle_value = vars[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          cout << "steering_angle: " << steer_value / deg2rad(25) << endl;
          cout << "throttle: " << throttle_value << endl;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 2; i < vars.size(); i++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (double i = 0; i < 100; i++){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
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
