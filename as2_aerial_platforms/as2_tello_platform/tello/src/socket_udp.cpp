#include "socket_udp.hpp"

SocketUdp::SocketUdp(const std::string& host, int port, uint bufferSize) {
  std::cout << "Creating socket ..." << std::endl;
  host_ = host;
  port_ = port;

  /* socket: create the socket */
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cout << "Error opening socket..." << std::endl;
  }

  /* socket: set reusable */
  const int enable = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    std::cout << "setsockopt(SO_REUSEADDR) failed" << std::endl;
  }

  /* build the server's Internet address */
  serv_addr_.sin_port        = htons(port_);
  serv_addr_.sin_addr.s_addr = inet_addr(host_.c_str());
  serv_addr_.sin_family      = AF_INET;

  /* build the sending destination addres */
  if (!setDestAddr()) {
    std::cout << "Unable to setDestAddr" << std::endl;
  }
}

SocketUdp::~SocketUdp() {
  std::cout << "closing socket ..." << std::endl;
  close(socket_fd_);
}

bool SocketUdp::setDestAddr() {
  addrinfo* addrInfo{nullptr};
  addrinfo hints{};
  hints.ai_family      = AF_INET;
  hints.ai_socktype    = SOCK_DGRAM;
  std::string port_str = std::to_string(port_);
  int ret              = getaddrinfo(host_.c_str(), port_str.c_str(), &hints, &addrInfo);
  if (ret != 0) {
    std::cout << "Error: setting dest_addr sockaddr_storage" << std::endl;
    return false;
  }
  memcpy(&dest_addr_, addrInfo->ai_addr, addrInfo->ai_addrlen);
  freeaddrinfo(addrInfo);
  return true;
}

bool SocketUdp::bindServer() {
  std::cout << "Server binding to " << host_ << ":" << port_ << std::endl;
  int ret = bind(socket_fd_, reinterpret_cast<sockaddr*>(&serv_addr_), sizeof(serv_addr_));
  if (ret < 0) {
    std::cout << "Unable to bind." << std::endl;
    return false;
  }
  return true;
}

bool SocketUdp::sending(std::string message) {
  const std::vector<unsigned char> msgs{std::cbegin(message), std::cend(message)};
  const socklen_t dest_addr_len{sizeof(dest_addr_)};

  int n = sendto(socket_fd_, msgs.data(), msgs.size(), 0, reinterpret_cast<sockaddr*>(&dest_addr_),
                 dest_addr_len);
  if (n < 0) {
    std::cout << "sending: It has been impossible to send the message " << message << std::endl;
    return false;
  }
  return true;
}

std::string SocketUdp::receiving(const int flags) {
  std::string msg;
  socklen_t serv_addr_len{sizeof(dest_addr_)};
  int n = recvfrom(socket_fd_, buffer_.data(), buffer_.size(), flags,
                   reinterpret_cast<sockaddr*>(&dest_addr_), &serv_addr_len);

  if (n < 1) {
    return "";
  }

  msg.append(buffer_.cbegin(), buffer_.cbegin() + n);
  msg = msg.erase(msg.find_last_not_of(" \n\r\t") + 1);
  return msg;
}
