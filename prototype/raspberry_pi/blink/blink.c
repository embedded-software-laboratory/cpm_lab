/*
 * blink.c:
 *      blinks the first LED
 *      Gordon Henderson, projects@drogon.net
 */

// complie with
// gcc -lwiringPi -o blink blink.c

#include <stdio.h>
#include <wiringPi.h>
#include <time.h>

#define PIN 29
int main (void)
{
  printf ("Raspberry Pi blink\n") ;

  if (wiringPiSetup () == -1)
    return 1 ;

  pinMode (PIN, OUTPUT) ;         // aka BCM_GPIO pin 17

  for (;;)
  {
    if(((unsigned long)time(NULL))%2) {
      digitalWrite (PIN, 1);
    }
    else {
      digitalWrite (PIN, 0);
    }

    nanosleep((const struct timespec[]){{0, 500000L}}, NULL);
  }
  return 0 ;
}
