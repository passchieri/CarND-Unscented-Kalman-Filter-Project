#include <iostream>
#include <iomanip>
#include <fstream>
#include <uWS/uWS.h>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"
#include "state.h"



using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


void process_sensorreading( string sensor_measurement, State &state) {
  
  // used to compute the RMSE later
  Tools tools;
  
  MeasurementPackage meas_package;
  istringstream iss(sensor_measurement);
  long long timestamp;
  
  
  string sensor_type;
  iss >> sensor_type;
  
  if (sensor_type.compare("L") == 0) {
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float px;
    float py;
    iss >> px;
    iss >> py;
    meas_package.raw_measurements_ << px, py;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
  } else if (sensor_type.compare("R") == 0) {
    
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro,theta, ro_dot;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
  }
  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  VectorXd gt_values(4);
  gt_values(0) = x_gt;
  gt_values(1) = y_gt;
  gt_values(2) = vx_gt;
  gt_values(3) = vy_gt;
  state.ground_truth.push_back(gt_values);
  
  //Call ProcessMeasurment(meas_package) for Kalman filter
  state.ukf.ProcessMeasurement(meas_package);
  
  //Push the current estimated x,y positon from the Kalman filter's state vector
  
  
  double p_x = state.ukf.x_(0);
  double p_y = state.ukf.x_(1);
  double v  = state.ukf.x_(2);
  double yaw = state.ukf.x_(3);
  
  double v1 = cos(yaw)*v;
  double v2 = sin(yaw)*v;
  
  VectorXd estimate = VectorXd(4);
  estimate(0) = p_x;
  estimate(1) = p_y;
  estimate(2) = v1;
  estimate(3) = v2;
  
  state.estimations.push_back(estimate);
  
  //std::cout << "\nRMSE calculation\nGT\n"<<gt_values <<endl;
  //std::cout << "EST\n" <<estimate<<endl;
  state.RMSE = tools.CalculateRMSE(state.estimations, state.ground_truth);
  std::cout <<state.RMSE.transpose()<<endl;
  if (meas_package.sensor_type_==MeasurementPackage::RADAR) {
    (state.nisFileRadar) << state.ukf.NIS_Radar<<endl;
  } else if (meas_package.sensor_type_==MeasurementPackage::LASER) {
    (state.nisFileLaser) << state.ukf.NIS_Laser<<endl;
  }
  
  (state.dataFile) << gt_values.transpose() << " "<<estimate.transpose() << " " << state.RMSE.transpose() << endl;
}

int readDataFromFile(char * fname) {
  UKF ukf;
  
  State state;
  ifstream infile;

 // string fname="../../data.txt";
  infile.open(fname);
  if (!infile) {
    cerr << "Unable to open file "<<fname;
    return 1; // terminate with error
  }
  string sensor_measurement;
  while (std::getline(infile,sensor_measurement)) {
    process_sensorreading(sensor_measurement,state);
    
  }
  
  infile.close();
  return 0;
}




int readDataFromWebsocket()
{
  uWS::Hub h;
  
  State state;
  
  h.onMessage([&state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          string sensor_measurement = j[1]["sensor_measurement"];
          
          process_sensorreading( sensor_measurement, state);
          VectorXd estimate=state.estimations.back();

          json msgJson;
          msgJson["estimate_x"] = estimate(0);
          msgJson["estimate_y"] = estimate(1);
          msgJson["rmse_x"] =  state.RMSE(0);
          msgJson["rmse_y"] =  state.RMSE(1);
          msgJson["rmse_vx"] = state.RMSE(2);
          msgJson["rmse_vy"] = state.RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    
  });
  
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
//  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
//    const std::string s = "<h1>Hello world!</h1>";
//    if (req.getUrl().valueLength == 1)
//    {
//      res->end(s.data(), s.length());
//    }
//    else
//    {
//      // i guess this should be done more gracefully?
//      res->end(nullptr, 0);
//    }
//  });
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cerr << "Connected!!!" << std::endl;
  });
  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
   // ws.close();
    std::cerr << "Disconnected" << std::endl;
    exit(0);
  });
  
  int port = 4567;
  if (h.listen(port))
  {
    std::cerr << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  return 0;
}

int main(int argc, char * argv[]) {
  if (argc<2) {
    return readDataFromWebsocket();
  } else {
    return readDataFromFile(argv[1]);
  }
}

























































































