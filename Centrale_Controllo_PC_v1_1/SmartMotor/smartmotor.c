#include <stdio.h>
#include <asm/types.h>
#include <errno.h>
#include <unistd.h>
#include <sys/select.h>
#include <string.h>

#include "../Rs232/rs232.h"

/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int send_command(int *, unsigned char, char *, char *);

int main()
{
  /* Init Rs232 Device */
  int rs232_device = -1;
  char rs232_buffer[255];

  rs232_device = com_open("/dev/ttyUSB0", 115200, 'N', 8, 1);

  if(rs232_device == -1)
  {
    perror("com_open");
    return 1;
  }
  else
  {
    printf("Init Rs232\t[OK]\n");
  }
  
  if(send_command(&rs232_device, 0x80, "RPA", rs232_buffer) == -1)
    perror("send_command");
  else
    printf("Received message: %s\n", rs232_buffer);

  if(send_command(&rs232_device, 0x80, "RUJA", rs232_buffer) == -1)
    perror("send_command");
  else
    printf("Received message: %s\n", rs232_buffer);

  return 0;
}

int send_command(int *device, unsigned char index, char * command, char *message_out)
{
  const long int timeout_sec = 0;
  const long int timeout_usec = 100000;
  
  char rs232_buffer_tx[255];
  char rs232_buffer[255];
  char message_buffer[255] = "";
  char *message_end;
  int message_end_position;

  unsigned char bytes_write = 0;
  int bytes_read;
  unsigned int done = 0;

  fd_set rd;
  struct timeval select_timeout;
  int nfds = 0;
  int select_result = -1;

  select_timeout.tv_sec = timeout_sec;
  select_timeout.tv_usec = timeout_usec;

  sprintf(rs232_buffer_tx, "%c%s\r", index, command);
  bytes_write = write(*device, rs232_buffer_tx, strlen(rs232_buffer_tx));

  if(bytes_write < 0)
    return -1;

  while(!done)
  {
    FD_ZERO(&rd);
    FD_SET(*device, &rd);

    nfds = max(nfds, *device);

    select_result = select(nfds + 1, &rd, NULL, NULL, &select_timeout);

    if((select_result == -1) && (errno != EAGAIN))
      return -1;

    /* Monitor rs232 device */
    if(*device > 0)
    {
      if(FD_ISSET(*device, &rd))
      {
        bytes_read = read(*device, rs232_buffer, sizeof(rs232_buffer));
        rs232_buffer[bytes_read] = '\0';

        if(bytes_read > 0)
        {
          message_end = strchr(rs232_buffer, 0x0d);

          if(message_end == NULL) //end character not found
            strcat(message_buffer, rs232_buffer);
          else
          {
            message_end_position = message_end - rs232_buffer;
            memmove(rs232_buffer, rs232_buffer, message_end_position);

            strcat(message_buffer, rs232_buffer);
            strcpy(message_out, message_buffer);
            
            flush_device_input(device);
            done = 1;
          }
        }
        else
          return -1;
      }
      else  //timeout
      {
        printf("No response to command\n");
        done = 1;
      }
    }// end if(*device > 0)
    else
      return -1;

    select_timeout.tv_sec = timeout_sec;
    select_timeout.tv_usec = timeout_usec;
    
  }//end while
    
  return 0;
}
