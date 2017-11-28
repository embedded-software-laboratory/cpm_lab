
#include <time.h>
#include <stdio.h>
int main() {
    while(1) {
        printf("%lu", ((unsigned long)time(NULL))%2); 
        fflush(stdout);
        nanosleep((const struct timespec[]){{0, 500000L}}, NULL);
    }
}