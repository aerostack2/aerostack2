#include "tello.hpp"

static std::vector<std::string> split(const std::string& target, char c) {
  std::string temp;
  std::stringstream stringstream{target};
  std::vector<std::string> result;

  while (std::getline(stringstream, temp, c)) {
    result.push_back(temp);
  }

  return result;
}

Tello::Tello() {
  commandSender_ = std::make_unique<SocketUdp>(IP_command, port_command);
  stateRecv_     = std::make_unique<SocketUdp>("0.0.0.0", port_state);
}

Tello::~Tello() {
  connected_ = false;
  if (stateThd_.joinable()) {
    stateThd_.join();
  }
  if (videoThd_.joinable()) {
    videoThd_.join();
  }
  std::cout << "Clean exit." << std::endl;
}

bool Tello::connect() {
  connected_ = sendCommand("command");
  if (!connected_) {
    std::cout << "Error: Connecting to Tello" << std::endl;
    return false;
  }
  std::cout << "Tello connection established!" << std::endl;
  stateRecv_->bindServer();
  update();

  stateThd_ = std::thread(&Tello::threadStateFnc, this);
  videoThd_ = std::thread(&Tello::streamVideo, this);
  return connected_;
}

bool Tello::sendCommand(const std::string& command) {
  uint cont             = 0;
  const int time_limit  = 5;
  std::string msgs_back = "";

  do {
    commandSender_->sending(command);
    sleep(1);  // FIXME
    msgs_back = commandSender_->receiving();
    cont++;
  } while ((msgs_back.length() == 0) && (cont <= time_limit));

  if (cont > time_limit) {
    std::cout << "The command '" << command << "' is not received." << std::endl;
    return false;
  }
  std::cout << command << " --> " << msgs_back << std::endl;
  return strcmp(msgs_back.c_str(), "ok") == 0;
}

void Tello::threadStateFnc() {
  bool resp;

  while (connected_) {
    resp = getState();
    if (resp) update();
    sleep(0.2);  // FIXME
  }
}

void Tello::streamVideo() {
  bool response = sendCommand("streamon");

  if (response) {
    cv::VideoCapture capture{URL_stream, cv::CAP_FFMPEG};
    cv::Mat frame;

    while (connected_) {
      capture >> frame;
      if (!frame.empty()) {
        frame_ = frame;
      }
    }
    capture.release();
  }
}

bool Tello::getState() {
  std::string msgs = stateRecv_->receiving();

  if (msgs.length() == 0) {
    return false;
  }
  return parseState(msgs, state_);
}

bool Tello::parseState(const std::string& data, std::array<double, 16>& state) {
  std::vector<std::string> values, values_;
  values = split(data, ';');

  if (values.size() != state.size()) {
    std::cout << "Error: Adding data to the 'state' attribute" << std::endl;
    return false;
  }

  int i = 0;
  for (auto& value : values) {
    if (value.size()) {
      values_  = split(value, ':');
      state[i] = stod(values_[1]);
      i++;
    }
  }
  return true;
}

void Tello::update() {
  orientation_.x = state_[0];
  orientation_.y = state_[1];
  orientation_.z = state_[2];

  velocity_.x = state_[3];
  velocity_.y = state_[4];
  velocity_.z = state_[5];

  timeOF     = state_[8];
  height_    = state_[9];
  battery_   = (int)state_[10];
  timeMotor  = state_[12];
  barometer_ = state_[11];

  acceleration_.x = state_[13];
  acceleration_.y = state_[14];
  acceleration_.z = state_[15];

  imu_[0] = orientation_;
  imu_[1] = velocity_;
  imu_[2] = acceleration_;
}

// Forward or backward move.
bool Tello::x_motion(double x) {
  bool response = true;
  std::string msg;
  if (x > 0) {
    msg      = "forward " + std::to_string(abs(x));
    response = sendCommand(msg);
  } else if (x < 0) {
    msg      = "back " + std::to_string(abs(x));
    response = sendCommand(msg);
  }
  return response;
}

// right or left move.
bool Tello::y_motion(double y) {
  bool response = true;
  std::string msg;
  if (y > 0) {
    msg      = "right " + std::to_string(abs(y));
    response = sendCommand(msg);
  } else if (y < 0) {
    msg      = "left " + std::to_string(abs(y));
    response = sendCommand(msg);
  }
  return response;
}

// up or left down.
bool Tello::z_motion(double z) {
  bool response = true;
  std::string msg;
  if (z > 0) {
    msg      = "up " + std::to_string(int(abs(z)));
    response = sendCommand(msg);
  } else if (z < 0) {
    msg      = "down " + std::to_string(int(abs(z)));
    response = sendCommand(msg);
  }
  return response;
}

// clockwise or counterclockwise
bool Tello::yaw_twist(double yaw) {
  bool response = true;
  std::string msg;
  if (yaw > 0) {
    msg      = "cw " + std::to_string(abs(yaw));
    response = sendCommand(msg);
  } else if (yaw < 0) {
    msg      = "ccw " + std::to_string(abs(yaw));
    response = sendCommand(msg);
  }
  return response;
}

// speed motion
bool Tello::speedMotion(double x, double y, double z, double yaw) {
  bool response;
  std::string msg;

  msg = "rc " + std::to_string(int(x)) + " " + std::to_string(int(y)) + " " +
        std::to_string(int(z)) + " " + std::to_string(int(yaw));
  response = sendCommand(msg);

  return response;
}