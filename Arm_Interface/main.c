#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>
#include <errno.h>

#include <string.h>

#include "arm_udp.h"
#include "rs232.h"

#define ARM_PORT 8012
#define JOY_ADDRESS "192.168.1.60"
#define JOY_PORT 8013

/* Macro */
#undef max
#define max(x,y) ((x) > (y) ? (x) : (y))

/* Prototype */
int main()
{
  
  /* Arm interface */
  int arm_client = -1;
  struct arm_frame arm_buffer_temp;
  struct sockaddr_in arm_client_address_dest;
  socklen_t arm_client_address_dest_len = sizeof(arm_client_address_dest);
  char *arm_token_result;
	
  /* rs232 interface */
  int arm_rs232_device = -1;
  unsigned char arm_rs232_buffer_temp[RS232_BUFFER_SIZE];
  
  /* Generic Variable */
  int done = 0;  // for the while in main loop
  int bytes_read;  // to check how many bytes has been read
  int bytes_sent;

  int select_result = -1;  // value returned frome select()
  int nfds = 0;  // fd to pass to select()
  fd_set rd, wr, er; // structure for select()
  struct timeval select_timeout;
  struct timeval *select_timeout_pointer;
  
  /* Peripheral initialization */

   /* Init Arm Interface */
  if(arm_open_server(&arm_client, &arm_client_address_dest, ARM_PORT) == 0)
    perror("error connection");
  else
    printf("Init arm client\t[OK]\n");

  /* Init rs232 Interface */
  arm_rs232_device = com_open("/dev/ttyUSB1", 115200, 'N', 8, 1);

  if(arm_rs232_device == -1)
    perror("com_open");
  else
    printf("Init rs232\t[OK]\n");

  printf("Run main program. . .\n");

  while(!done)
  {
    fflush(stdout);

    FD_ZERO(&rd);
    FD_ZERO(&wr);
    FD_ZERO(&er);

    if(arm_client > 0)
    {
      FD_SET(arm_client, &rd);
      nfds = max(nfds, arm_client);  

      if(arm_buffer_tx_empty == 0)
      {
        FD_SET(arm_client, &wr);
        nfds = max(nfds, arm_client);
      }
    }
	
    if(arm_rs232_device > 0)
    {
      FD_SET(arm_rs232_device, &rd);
      nfds = max(nfds, arm_rs232_device);  

      if(rs232_buffer_tx_empty == 0)
      {
        FD_SET(arm_rs232_device, &wr);
        nfds = max(nfds, arm_rs232_device);
      }
    }
	
    select_result = select(nfds + 1, &rd, &wr, NULL, NULL);

    if(select_result == -1 && errno == EAGAIN)
    {
      perror("select");
      continue;
    }

    if(select_result == -1)
    {
      perror("main:");
      return 1;
    }

    /* Manage rs232 message */
    if(arm_rs232_device > 0)
    {
      if(FD_ISSET(arm_rs232_device, &rd))
      {
        bytes_read = rs232_read(arm_rs232_device);

        if(bytes_read < 0)
          perror("rs232 read");

        // load udp buffer only if I have received the whole message
        if(rs232_check_last_char(0x0d))
        {
          bytes_read = rs232_unload_rx(arm_rs232_buffer_temp);
  
          if(bytes_read > 0)
          {
            arm_rs232_buffer_temp[bytes_read] = 0;
            memcpy(arm_buffer_temp.param.arm_command, arm_rs232_buffer_temp, bytes_read + 1);
            arm_load_tx(arm_buffer_temp);
          }
        }
      } // end if(FD_ISSET(socket_can, &rd))

      if(FD_ISSET(arm_rs232_device, &wr))
      {
        bytes_sent = rs232_write(arm_rs232_device);
 
        if(bytes_sent <= 0)
          printf("Error on rs232_write");
/*	else
	{
          if((debug_timer > 0) && (rs232_buffer_tx_empty == 1))
          {
            clock_gettime(CLOCK_REALTIME, &debug_timer_stop);
            debug_timer = 0;
      
            debug_elapsed_time = debug_timer_stop.tv_nsec - debug_timer_start.tv_nsec;
            max_debug_elapsed_time = max(max_debug_elapsed_time, debug_elapsed_time);
            printf("Elapsed time: %ldns\n", debug_elapsed_time);
            printf("Max Elapsed time. %ldns; debug_timer: %i\n", max_debug_elapsed_time, debug_timer);
          } 
	}*/
      }
    }
    else
    {
      /* Init rs232 Interface */
      arm_rs232_device = com_open("/dev/ttyUSB1", 115200, 'N', 8, 1);

      if(arm_rs232_device > -1)
        printf("Init rs232\t[OK]\n");
    }
	
    /* Redirect message for arm */
    if(arm_client > 0)
    {
      if(FD_ISSET(arm_client, &rd))
      {
        // Ethernet is fastest than rs232, so I don't need circular buffer on rx
        bytes_read = recvfrom(arm_client, &arm_buffer_temp, sizeof(struct arm_frame), 0, (struct sockaddr *)&arm_client_address_dest, &arm_client_address_dest_len);

        if(bytes_read > 0)
        {
		// Every message from arm must ends with \r
          arm_token_result = strchr(arm_buffer_temp.param.arm_command, 13);
		  
		  if(arm_token_result != NULL)
		  {
		    *(arm_token_result + 1) = '\0';
            rs232_load_tx((__u8 *)arm_buffer_temp.param.arm_command, strlen(arm_buffer_temp.param.arm_command));
		  }
	  /*if(debug_timer == 0)
          {
            clock_gettime(CLOCK_REALTIME, &debug_timer_start);
            debug_timer++;
            printf("\033[2A");
          }*/
        }
  
      } //if(FD_ISSET(arm_client, &rd))

      if(FD_ISSET(arm_client, &wr))
      {
        //printf("%i\n", rs232_buffer_tx_data_count);
        bytes_sent = arm_send(arm_client, &arm_client_address_dest);

        if(bytes_sent <= 0)
          printf("Error on pc_interface_send");
      }  //if(FD_ISSET(pc_interface_client, &wr))
    }
  
  }  // end while(!= done)
  
  return 0;
}