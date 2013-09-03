#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

int main()
{
  int result;

  // Export gpio led in user space
  result = system("echo 60 > /sys/class/gpio/export"); 

  if(result < 0)
    perror("system");

  // Set gpio as output
  result = system("echo out > /sys/class/gpio/gpio60/direction");

  if(result < 0)
    perror("system");

  return 0;
}
