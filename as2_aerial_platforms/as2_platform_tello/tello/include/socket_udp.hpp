#ifndef SOCKETUDP_H
#define SOCKETUDP_H

#include <arpa/inet.h>   // For inet_addr()
#include <netdb.h>       // For gethostbyname()
#include <netinet/in.h>  // For sockaddr_in
#include <string.h>
#include <sys/socket.h>  // For socket(), connect(), send(), and recv()
#include <sys/types.h>   // For data types
#include <unistd.h>      // For close()

#include <stdio.h>

#include <cstring>  // memcpy
#include <sstream>

#include <array>
#include <iostream>
#include <string>
#include <vector>

class SocketUdp {
private:
  int socket_fd_;
  std::string host_;
  int port_;

  sockaddr_in serv_addr_;
  std::array<unsigned char, 1024> buffer_;
  sockaddr_storage dest_addr_;

private:
  bool setDestAddr();

public:
  SocketUdp(const std::string& host = "0.0.0.0", int port = 0, uint bufferSize = 1024);
  ~SocketUdp();  // closing socket

  bool bindServer();
  inline int getSocketfd() const { return socket_fd_; }
  inline const char* getIP() const { return host_.c_str(); }
  inline int getPort() const { return port_; }

  inline void setSocketfd(int socket_fd) { this->socket_fd_ = socket_fd; }
  inline void setIP(const char* host) { this->host_ = host; }
  inline void setPort(int port) { this->port_ = port; }

  bool sending(std::string message);
  std::string receiving(const int flags = MSG_DONTWAIT);
};

#endif  // SOCKETUDP_H
