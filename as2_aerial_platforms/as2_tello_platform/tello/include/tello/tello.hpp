#ifndef TELLO_H
#define TELLO_H

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
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
  double x, y, z;
};

class Tello {
private:
  std::thread stateThd_;
  std::thread videoThd_;

  std::unique_ptr<SocketUdp> commandSender_;
  std::unique_ptr<SocketUdp> stateRecv_;

  // State information.
  bool connected_;

  std::array<double, 16> state_;
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
  bool parseState(const std::string& data, std::array<double, 16>& state);
  void update();
  void threadStateFnc();

public:
  Tello();   // creating sockets
  ~Tello();  // closing sockets

  bool connect();

  bool getState();
  bool sendCommand(const std::string& command);

  inline bool isConnected() { return connected_; }
  inline std::array<coordinates, 3> getIMU() { return imu_; }
  inline coordinates getOrientation() { return orientation_; }
  inline coordinates getVelocity() { return velocity_; }
  inline coordinates getAcceleration() { return acceleration_; }
  inline double getBarometer() { return barometer_; }
  inline double getHeight() { return height_; }
  inline double getBattery() { return battery_; }
  inline cv::Mat getFrame() { return frame_; }

  void streamVideo();

  bool x_motion(double x);                                     // Forward or backward move.
  bool y_motion(double y);                                     // right or left move.
  bool z_motion(double z);                                     // up or left down.
  bool yaw_twist(double yaw);                                  // clockwise or counterclockwise
  bool speedMotion(double x, double y, double z, double yaw);  //
};

#endif  // TELLO_H