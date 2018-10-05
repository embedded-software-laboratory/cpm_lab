#pragma once

// This should be temporary, use DDS/RTPS for proper implementation!

  #include <netinet/in.h>
  #include <sys/socket.h>


typedef struct sockaddr_in ip_addr;
int make_socket();
void send_to(int sock, ip_addr target, void* message, size_t n_bytes);
ip_addr make_addr(char* ip, int port);
int receive(int sock, char* sender_ip_out, char* message_out, size_t message_max_size);