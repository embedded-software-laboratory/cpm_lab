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

#define BUFLEN 512  //Max length of buffer
 
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

int main ()
{


    struct sockaddr_in si_other;
    int s, i, slen=sizeof(si_other);
    char buf[BUFLEN];
    char message[BUFLEN];
 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
 
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(4210);


    string hostname = "esp8266_43243.local";

    struct hostent * hst = gethostbyname(hostname.c_str());
    if(!hst) {
        die("gethostbyname");
    }

    if(hst->h_addr_list == NULL) {
        die("hst->h_addr_list");        
    }

    char addr_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, hst->h_addr_list[0], addr_str, INET_ADDRSTRLEN);
    cout << hostname << " == " << addr_str << endl;


    if (inet_aton(addr_str , &si_other.sin_addr) == 0) 
    {
        die("inet_aton");
    }
 
    FILE *kbd = fopen("/dev/input/by-path/platform-i8042-serio-0-event-kbd", "r");


    while(1)
    {



        char key_map[KEY_MAX/8 + 1];    //  Create a byte array the size of the number of keys

        memset(key_map, 0, sizeof(key_map));    //  Initate the array to zero's
        ioctl(fileno(kbd), EVIOCGKEY(sizeof(key_map)), key_map);    //  Fill the keymap with the current keyboard state



        /*
        printf("Enter message : ");
        fgets(message,BUFLEN-1,stdin);
         */


        char steering = 127;
        char throttle = 127;

        if(check_key(key_map, KEY_LEFT)) steering = 255;
        if(check_key(key_map, KEY_RIGHT)) steering = 0;
        if(check_key(key_map, KEY_UP)) throttle = 255;
        if(check_key(key_map, KEY_DOWN)) throttle = 0;

        message[0] = steering;
        message[1] = throttle;

        //send the message
        if (sendto(s, message, 2 , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
        usleep(20000);
        

        /* 
        //receive a reply and print it
        //clear the buffer by filling null, it might have previously received data
        memset(buf,'\0', BUFLEN);
        //try to receive some data, this is a blocking call
        if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, (socklen_t*)(&slen)) == -1)
        {
            die("recvfrom()");
        }
         
        puts(buf);*/
    }





/*


    FILE *kbd = fopen("/dev/input/by-path/platform-i8042-serio-0-event-kbd", "r");

    while (true){
        char key_map[KEY_MAX/8 + 1];    //  Create a byte array the size of the number of keys

        memset(key_map, 0, sizeof(key_map));    //  Initate the array to zero's
        ioctl(fileno(kbd), EVIOCGKEY(sizeof(key_map)), key_map);    //  Fill the keymap with the current keyboard state

        if(check_key(key_map, KEY_LEFT)) cout << "l";
        if(check_key(key_map, KEY_RIGHT)) cout << "r";
        if(check_key(key_map, KEY_UP)) cout << "u";
        if(check_key(key_map, KEY_DOWN)) cout << "d";

        cout << endl;
        usleep(100000);
    }*/

}