#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <linux/input.h>       
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>

using namespace std;

void die(string s)
{
    perror(s.c_str());
    exit(1);
}

bool check_key(char* key_map, int key) {
    int keyb = key_map[key/8]; 
    int mask = 1 << (key % 8);
    return (keyb & mask);
}

int main (int argc, char** argv)
{


    struct sockaddr_in si_other;
    int s, i, slen=sizeof(si_other);
 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        die("socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(6783);


    /*string hostname = "esp8266_44142.local";

    struct hostent * hst = gethostbyname(hostname.c_str());
    if(!hst) {
        die("gethostbyname");
    }

    if(hst->h_addr_list == NULL) {
        die("hst->h_addr_list");        
    }

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, hst->h_addr_list[0], addr_str, INET_ADDRSTRLEN);
    cout << hostname << " == " << addr_str << endl;*/

    char addr_str[] = "192.168.0.100";

    if (inet_aton(addr_str , &si_other.sin_addr) == 0)  {
        die("inet_aton");
    }
 
    //FILE *kbd = fopen("/dev/input/by-id/usb-413c_Dell_KB216_Wired_Keyboard-event-kbd", "r");
    //FILE *kbd = fopen("/dev/input/by-id/usb-E-Signal_USB_Gaming_Mouse-if01-event-kbd", "r");
    FILE *kbd = fopen("/dev/input/by-id/usb-DELL_Dell_QuietKey_Keyboard-event-kbd", "r");


    float curvature = 0;
    const float delta_curvature = 0.3;
    const float max_curvature = 2.4;
    const float eps = 1e-5;

    while(1) {
        char key_map[KEY_MAX/8 + 1];    //  Create a byte array the size of the number of keys
        memset(key_map, 0, sizeof(key_map));    //  Initate the array to zero's
        ioctl(fileno(kbd), EVIOCGKEY(sizeof(key_map)), key_map);    //  Fill the keymap with the current keyboard state

        float speed = 0;
        if(check_key(key_map, KEY_LEFT))  {
            curvature += delta_curvature;
        }
        else if(check_key(key_map, KEY_RIGHT))  {
            curvature -= delta_curvature;
        }
        else {
            if(curvature > delta_curvature - eps) {
                curvature -= delta_curvature;
            }
            else if(curvature < -delta_curvature + eps) {
                curvature += delta_curvature;
            }
        }

        if(curvature > max_curvature) {
            curvature = max_curvature;
        }
        else if(curvature < -max_curvature) {
            curvature = -max_curvature;
        }


        if(check_key(key_map, KEY_UP)) speed = 0.9;
        if(check_key(key_map, KEY_DOWN)) speed = -0.9;

        char message[9];
        message[0] = 'c';
        memcpy(message+1, &speed, 4);
        memcpy(message+5, &curvature, 4);
        
        


        if (sendto(s, message, 9 , 0 , (struct sockaddr *) &si_other, slen)==-1) {
            die("sendto()");
        }
        printf("sent: %f, %f\n", speed, curvature);
        usleep(40000);
    }
}