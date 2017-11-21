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


#define SERVER "192.168.0.103"
#define BUFLEN 512  //Max length of buffer
#define PORT 4210   //The port on which to send data
 
void die(char *s)
{
    perror(s);
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
    si_other.sin_port = htons(PORT);
     
    if (inet_aton(SERVER , &si_other.sin_addr) == 0) 
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
 
    while(1)
    {
        printf("Enter message : ");
        gets(message);
         
        //send the message
        if (sendto(s, message, strlen(message) , 0 , (struct sockaddr *) &si_other, slen)==-1)
        {
            die("sendto()");
        }
        

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