#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define SYS_5V 44
#define SYS_ENABLE 45

volatile int STOP = 0;

int gpio_export(int pin_number)
{
  FILE *file = NULL;

  file = fopen("/sys/class/gpio/export", "a");

  if(file == NULL)
    return -1;

  fprintf(file, "%i", pin_number);

  fclose(file);
  return 1;
}

int gpio_set_value(int pin_number, int value)
{
  FILE *file = NULL;
  char file_path[64];

  sprintf(file_path, "/sys/class/gpio/gpio%i/value", pin_number);
  file = fopen(file_path, "a");

  if(file == NULL)
    return -1;

  fprintf(file, "%i", value);

  fclose(file);
  return 1;
}

int gpio_set_direction(int pin_number, int value)
{
  FILE *file = NULL;
  char file_path[64];

  sprintf(file_path, "/sys/class/gpio/gpio%i/direction", pin_number);
  file = fopen(file_path, "a");

  if(file == NULL)
    return -1;

  if(value)
    fprintf(file, "in");
  else
    fprintf(file, "out");

  fclose(file);
  return 1;
}

int main()
{
  int status = 0;
  struct timeval select_timeout;

  int fd, res;
  char buf[255];
  
  // set gpio as input pulled down
  /*printf("Setting gpio as input\n");
  sprintf(buf, "echo 27 > /sys/kernel/debug/omap_mux/gpmc_ad12");
  if(system(buf) < 0)
    perror("setting gpio as input");*/
  
  // export gpio
  printf("Export gpio\n");

  gpio_export(SYS_5V);
  gpio_set_direction(SYS_5V, 1);
	
  // set gpio as output
  printf("Setting gpio as input\n");
  /*sprintf(buf, "echo 17 > /sys/kernel/debug/omap_mux/gpmc_ad13");
  if(system(buf) < 0)
    perror("setting gpio as output");*/
  
  // export gpio
  printf("Export gpio\n");
  gpio_export(SYS_ENABLE);
  printf("Setting gpio as output\n");
  gpio_set_direction(SYS_ENABLE, 0);
  /*sprintf(buf, "echo %i > /sys/class/gpio/export", SYS_ENABLE);
  if(system(buf) < 0)
    perror("export gpio");*/

  select_timeout.tv_sec = 0;
  select_timeout.tv_usec = 500000;

  // Enable battery
  /*sprintf(buf, "echo 1 > /sys/class/gpio/gpio%i/value", SYS_ENABLE);
  if(system(buf) < 0)
    perror("enable power");*/
  gpio_set_value(SYS_ENABLE, 1);
	
  while(STOP == 0)
  {
    status = select(1, NULL, NULL, NULL, &select_timeout);

    sprintf(buf, "/sys/class/gpio/gpio%i/value", SYS_5V);
    fd = open(buf, O_RDWR | O_NOCTTY);
	
    res = read(fd, buf, 255);
    buf[res] = 0;

    if(res < 0)
      perror("read");

    if(buf[0] == '0')
    {
      printf("Turning off the computer. . . \n");
      system("shutdown -P now");

      STOP = 1;
    }
  
    close(fd);
	
    select_timeout.tv_sec = 1;
    select_timeout.tv_usec = 0;
  }

  return 0;
}


