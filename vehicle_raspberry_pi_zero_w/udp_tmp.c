#include <stdint.h>
#include <linux/joystick.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <linux/input.h>       
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>
#include "udp_tmp.h"


// This should be temporary, use DDS/RTPS for proper implementation!



void die(char* s)
{
    perror(s);
    exit(1);
}




int setNonblocking(int fd)
{
    int flags;
    if (-1 == (flags = fcntl(fd, F_GETFL, 0)))
        flags = 0;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}  





int make_socket() {
    int sock;
    if ( (sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        die("socket");
    }

    struct sockaddr_in si_me;
    memset((char *) &si_me, 0, sizeof(si_me));
     
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(6783);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
    //bind socket to port
    if( bind(sock, (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    setNonblocking(sock);
    return sock;
}


int receive(int sock, char* sender_ip_out, char* message_out, size_t message_max_size) {
    ip_addr si_other;
    unsigned int slen = sizeof(ip_addr);
    int recv_len = recvfrom(sock, message_out, message_max_size, 0, (struct sockaddr *) &si_other, &slen);

    if(recv_len > 1) {
        inet_ntop(AF_INET, &(si_other.sin_addr), sender_ip_out, 1000);
    }

    return recv_len;
}



ip_addr make_addr(char* ip, int port) {
    ip_addr target_addr;
    memset((char *) &target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    if (inet_aton(ip , &target_addr.sin_addr) == 0)  {
        die("inet_aton");
    }
    return target_addr;
}



void send_to(int sock, ip_addr target, void* message, size_t n_bytes) {
    if (sendto(sock, message, n_bytes , 0 , (struct sockaddr *) &target, sizeof(ip_addr))==-1) {
        die("sendto()");
    }
}


