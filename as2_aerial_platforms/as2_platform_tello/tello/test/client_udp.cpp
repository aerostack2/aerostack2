// Client side implementation of UDP client-server model

#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>

#define PORT 8889
#define MAXLINE 1024

// Driver code
int main() {
  int sockfd;
  char buffer[MAXLINE];
  std::vector<unsigned char> buffer2;
  buffer2.resize(MAXLINE, '\0');
  char *hello = "command";
  sockaddr_in servaddr;
  char *IP = "192.168.10.1";
  // Creating socket file descriptor
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  memset(&servaddr, 0, sizeof(servaddr));

  // Filling server information
  servaddr.sin_family      = AF_INET;
  servaddr.sin_port        = htons(PORT);
  servaddr.sin_addr.s_addr = inet_addr(IP);

  int n, len;

  sendto(sockfd, (const char *)hello, strlen(hello), MSG_CONFIRM,
         reinterpret_cast<sockaddr *>(&servaddr), sizeof(servaddr));
  std::cout << "Hello message sent." << std::endl;
  socklen_t *addrlen;
  n         = recvfrom(sockfd, buffer2.data(), MAXLINE, MSG_WAITALL,
               reinterpret_cast<sockaddr *>(&servaddr), addrlen);
  buffer[n] = '\0';
  std::cout << "Server : ";
  for (int i = 0; i < buffer2.size(); i++) {
    std::cout << buffer2[i];
  }
  std::cout << std::endl;
  close(sockfd);
  return 0;
}
