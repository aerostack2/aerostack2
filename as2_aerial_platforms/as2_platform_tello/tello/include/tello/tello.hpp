#ifndef TELLO_H
#define TELLO_H

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "socket_udp.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

const int port_command = 8889;
const int port_state   = 8890;
// const char* const IP_command{"127.0.0.1"};
const char* const IP_command{"192.168.10.1"};
const char* const URL_stream{"udp://0.0.0.0:11111"};

struct coordinates {
  double x = 0;
  double y = 0;
  double z = 0;
};

class Tello {
private:
  std::mutex state_mutex_;
  std::thread stateThd_;
  std::thread videoThd_;

  std::unique_ptr<SocketUdp> commandSender_;
  std::unique_ptr<SocketUdp> stateRecv_;

  // State information.
  bool connected_;

  // std::array<double, 16> state_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::unordered_map<std::string, double> state_{
      {"pitch", 0.0}, {"roll", 0.0},  {"yaw", 0.0}, {"vgx", 0.0}, {"vgy", 0.0}, {"vgz", 0.0},
      {"templ", 0.0}, {"temph", 0.0}, {"tof", 0.0}, {"h", 0.0},   {"bat", 0.0}, {"baro", 0.0},
      {"time", 0.0},  {"agx", 0.0},   {"agy", 0.0}, {"agz", 0.0}};
  coordinates orientation_;
  coordinates velocity_;
  coordinates acceleration_;

  double battery_;
  double timeMotor;
  double timeOF;
  double height_;
  double barometer_;

  std::array<coordinates, 3> imu_;

  cv::Mat frame_;

private:
  bool parseState(const std::string& data);
  void update();
  void threadStateFnc();

public:
  Tello();   // creating sockets
  ~Tello();  // closing sockets

  bool connect();

  bool getState();
  // bool sendCommand(const std::string& command, bool wait = true);
  bool sendCommand(const std::string& command, bool wait = true, std::string* response = nullptr);

  inline bool isConnected() { return connected_; }
  inline std::array<coordinates, 3> getIMU() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return imu_;
  }
  inline coordinates getOrientation() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return orientation_;
  }
  inline coordinates getVelocity() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return velocity_;
  }
  inline coordinates getAcceleration() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return acceleration_;
  }
  inline double getBarometer() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return barometer_;
  }
  inline double getHeight() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return height_;
  }
  inline double getBattery() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return battery_;
  }
  inline cv::Mat getFrame() { return frame_; }

  void streamVideo();

  bool x_motion(double x);                                     // Forward or backward move.
  bool y_motion(double y);                                     // right or left move.
  bool z_motion(double z);                                     // up or left down.
  bool yaw_twist(double yaw);                                  // clockwise or counterclockwise
  bool speedMotion(double x, double y, double z, double yaw);  //
};

#endif  // TELLO_H
